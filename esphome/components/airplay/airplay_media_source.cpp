#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

#include <mdns.h>

#include <cinttypes>
#include <cstdio>

namespace esphome::airplay {

ESPHOME_LOG_TAG(TAG, "airplay");

static constexpr char URI_PREFIX[] = "airplay://";
static constexpr char PLAY_URI[] = "airplay://current";

static constexpr uint32_t RTSP_TASK_STACK_SIZE = 8192;
static constexpr uint32_t AUDIO_TASK_STACK_SIZE = 6144;
static constexpr UBaseType_t RTSP_TASK_PRIORITY = 3;
static constexpr UBaseType_t AUDIO_TASK_PRIORITY = 5;

static constexpr uint32_t MDNS_RETRY_ID = 0;
static constexpr uint32_t MDNS_RETRY_INTERVAL_MS = 5000;

void AirPlayMediaSource::setup() {
  get_mac_address_raw(this->mac_);

  if (!this->crypto_.init()) {
    ESP_LOGE(TAG, "Failed to load the RSA key");
    this->mark_failed();
    return;
  }
  if (!this->decoder_.init(FRAMES_PER_PACKET)) {
    ESP_LOGE(TAG, "Failed to allocate the decoder");
    this->mark_failed();
    return;
  }
  mbedtls_aes_init(&this->aes_);

  // Audio is buffered before playback starts; extra slots absorb packets that arrive early or out of order
  this->start_frames_ = this->buffer_duration_ms_ * SAMPLE_RATE / 1000;
  const uint32_t buffer_packets = (this->start_frames_ + FRAMES_PER_PACKET - 1) / FRAMES_PER_PACKET;
  this->slot_count_ = buffer_packets + buffer_packets / 4 + 8;

  RAMAllocator<PacketSlot> slot_allocator;
  this->slots_ = slot_allocator.make_unique_array_for_overwrite(this->slot_count_);
  RAMAllocator<uint8_t> byte_allocator(RAMAllocator<uint8_t>::PREFER_INTERNAL);
  this->packet_buffer_ = byte_allocator.make_unique_array_for_overwrite(PACKET_BUFFER_SIZE);
  this->decrypt_buffer_ = byte_allocator.make_unique_array_for_overwrite(PACKET_BUFFER_SIZE);
  // One spare frame, used when a frame is repeated to correct clock drift
  RAMAllocator<int16_t> sample_allocator(RAMAllocator<int16_t>::PREFER_INTERNAL);
  this->output_buffer_ = sample_allocator.make_unique_array_for_overwrite((FRAMES_PER_PACKET + 1) * NUM_CHANNELS);
  RAMAllocator<char> char_allocator(RAMAllocator<char>::PREFER_INTERNAL);
  // One extra byte per connection so that headers and body can be null terminated in place
  this->rtsp_buffers_ = char_allocator.make_unique_array_for_overwrite(MAX_RTSP_CONNECTIONS * (RTSP_BUFFER_SIZE + 1));

  if (!this->slots_ || !this->packet_buffer_ || !this->decrypt_buffer_ || !this->output_buffer_ ||
      !this->rtsp_buffers_) {
    ESP_LOGE(TAG, "Failed to allocate buffers");
    this->mark_failed();
    return;
  }
  for (uint32_t i = 0; i < this->slot_count_; i++)
    this->slots_[i].valid = false;
  for (size_t i = 0; i < MAX_RTSP_CONNECTIONS; i++)
    this->connections_[i].buffer = &this->rtsp_buffers_[i * (RTSP_BUFFER_SIZE + 1)];

  this->audio_events_ = xEventGroupCreate();
  if (this->audio_events_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
    return;
  }
  xEventGroupSetBits(this->audio_events_, AUDIO_EVENT_STOPPED);

  if (!this->audio_task_.create(AirPlayMediaSource::audio_task, "airplay_audio", AUDIO_TASK_STACK_SIZE, this,
                                AUDIO_TASK_PRIORITY, this->task_stack_in_psram_) ||
      !this->rtsp_task_.create(AirPlayMediaSource::rtsp_task, "airplay_rtsp", RTSP_TASK_STACK_SIZE, this,
                               RTSP_TASK_PRIORITY, this->task_stack_in_psram_)) {
    ESP_LOGE(TAG, "Failed to create tasks");
    this->mark_failed();
    return;
  }

  this->register_mdns_service_();
  this->disable_loop();
}

void AirPlayMediaSource::register_mdns_service_() {
  char instance[64];
  snprintf(instance, sizeof(instance), "%02X%02X%02X%02X%02X%02X@%s", this->mac_[0], this->mac_[1], this->mac_[2],
           this->mac_[3], this->mac_[4], this->mac_[5], this->name_);

  // cn: codecs (0 = PCM, 1 = ALAC), et: encryption types (0 = none, 1 = RSA)
  mdns_txt_item_t txt[] = {
      {"txtvers", "1"}, {"ch", "2"},    {"cn", "0,1"},   {"et", "0,1"},   {"ek", "1"},
      {"sv", "false"},  {"da", "true"}, {"sr", "44100"}, {"ss", "16"},    {"pw", "false"},
      {"sm", "false"},  {"tp", "UDP"},  {"vn", "3"},     {"vs", "105.1"}, {"am", "ESPHome"},
  };
  const esp_err_t err = mdns_service_add(instance, "_raop", "_tcp", this->port_, txt, sizeof(txt) / sizeof(txt[0]));
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to register the mDNS service, retrying: %s", esp_err_to_name(err));
    this->set_timeout(MDNS_RETRY_ID, MDNS_RETRY_INTERVAL_MS, [this]() { this->register_mdns_service_(); });
    return;
  }
  ESP_LOGD(TAG, "Registered mDNS service %s", instance);
}

void AirPlayMediaSource::dump_config() {
  ESP_LOGCONFIG(TAG,
                "AirPlay Media Source:\n"
                "  Name: %s\n"
                "  Port: %u\n"
                "  Buffer duration: %" PRIu32 " ms (%" PRIu32 " packets)\n"
                "  Task stack in PSRAM: %s",
                this->name_, this->port_, this->buffer_duration_ms_, this->slot_count_,
                YESNO(this->task_stack_in_psram_));
}

// THREAD CONTEXT: Main loop. Woken by set_session_state_() from the RTSP and audio tasks.
void AirPlayMediaSource::loop() {
  const auto desired = static_cast<SessionState>(this->session_state_.load(std::memory_order_acquire));
  const media_source::MediaSourceState state = this->get_state();

  switch (desired) {
    case SESSION_STATE_NONE:
      this->play_requested_ = false;
      if (state != media_source::MediaSourceState::IDLE)
        this->set_state_(media_source::MediaSourceState::IDLE);
      break;
    case SESSION_STATE_PLAYING:
    case SESSION_STATE_PAUSED:
      if (state == media_source::MediaSourceState::IDLE) {
        if (!this->play_requested_) {
          // Ask the orchestrator to switch to this source; it then calls play_uri()
          this->play_requested_ = true;
          this->request_play_uri_(PLAY_URI);
        }
      } else if (desired == SESSION_STATE_PLAYING && state == media_source::MediaSourceState::PAUSED) {
        this->set_state_(media_source::MediaSourceState::PLAYING);
      } else if (desired == SESSION_STATE_PAUSED && state == media_source::MediaSourceState::PLAYING) {
        this->set_state_(media_source::MediaSourceState::PAUSED);
      }
      break;
  }
  this->disable_loop();
}

// THREAD CONTEXT: RTSP or audio task
void AirPlayMediaSource::set_session_state_(SessionState state) {
  if (this->session_state_.exchange(state, std::memory_order_acq_rel) != state)
    this->enable_loop_soon_any_context();
}

bool AirPlayMediaSource::can_handle(const std::string &uri) const { return uri.starts_with(URI_PREFIX); }

// THREAD CONTEXT: Main loop
bool AirPlayMediaSource::play_uri(const std::string &uri) {
  this->play_requested_ = false;
  if (this->is_failed() || !this->can_handle(uri))
    return false;

  const auto desired = static_cast<SessionState>(this->session_state_.load(std::memory_order_acquire));
  if (desired == SESSION_STATE_NONE) {
    ESP_LOGW(TAG, "Cannot play: no AirPlay sender is connected");
    return false;
  }
  this->set_state_(desired == SESSION_STATE_PAUSED ? media_source::MediaSourceState::PAUSED
                                                   : media_source::MediaSourceState::PLAYING);
  return true;
}

// THREAD CONTEXT: Main loop
void AirPlayMediaSource::handle_command(media_source::MediaSourceCommand command) {
  const bool connected = this->session_state_.load(std::memory_order_acquire) != SESSION_STATE_NONE;
  uint8_t request = 0;
  switch (command) {
    case media_source::MediaSourceCommand::STOP:
      if (!connected) {
        this->play_requested_ = false;
        this->set_state_(media_source::MediaSourceState::IDLE);
        return;
      }
      request = REMOTE_REQUEST_DISCONNECT;
      break;
    case media_source::MediaSourceCommand::PLAY:
      request = REMOTE_REQUEST_PLAY;
      break;
    case media_source::MediaSourceCommand::PAUSE:
      request = REMOTE_REQUEST_PAUSE;
      break;
    case media_source::MediaSourceCommand::NEXT:
      request = REMOTE_REQUEST_NEXT;
      break;
    case media_source::MediaSourceCommand::PREVIOUS:
      request = REMOTE_REQUEST_PREVIOUS;
      break;
    default:
      return;
  }
  if (!connected)
    return;
  this->remote_requests_.fetch_or(request, std::memory_order_acq_rel);
}

// THREAD CONTEXT: Speaker playback callback
void AirPlayMediaSource::notify_audio_played(uint32_t frames, int64_t timestamp) {
  this->frames_played_.fetch_add(frames, std::memory_order_relaxed);
}

}  // namespace esphome::airplay

#endif  // USE_ESP32

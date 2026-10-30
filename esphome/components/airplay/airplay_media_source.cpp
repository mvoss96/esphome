#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif

#include <esp_heap_caps.h>
#include <mdns.h>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstring>

namespace esphome::airplay {

ESPHOME_LOG_TAG(TAG, "airplay");

static constexpr char URI_PREFIX[] = "airplay://";
static constexpr char PLAY_URI[] = "airplay://current";

// The RSA operations of a new session need most of this stack
static constexpr uint32_t AIRPLAY_TASK_STACK_SIZE = 8192;
static constexpr uint32_t REMOTE_TASK_STACK_SIZE = 4096;
static constexpr UBaseType_t REMOTE_TASK_PRIORITY = 2;
static constexpr UBaseType_t AIRPLAY_TASK_PRIORITY = 5;
static constexpr uint32_t LISTEN_RETRY_MS = 1000;
/// Longest wait for network traffic; the receiver shortens it while a session plays
static constexpr uint32_t RUN_INTERVAL_MS = 100;

/// The sender's volume slider is not updated for changes smaller than this
static constexpr float VOLUME_CHANGE_THRESHOLD = 0.005f;

static constexpr uint32_t MDNS_RETRY_ID = 0;
static constexpr uint32_t PLAY_RETRY_ID = 1;
static constexpr uint32_t PLAY_RETRY_MS = 3000;
static constexpr uint32_t MDNS_RETRY_INTERVAL_MS = 5000;

static constexpr size_t MAX_TXT_RECORDS = 20;

static void library_log(::airplay::LogLevel level, const char *message) {
  switch (level) {
    case ::airplay::LogLevel::LOG_LEVEL_ERROR:
      ESP_LOGE(TAG, "%s", message);
      break;
    case ::airplay::LogLevel::LOG_LEVEL_WARN:
      ESP_LOGW(TAG, "%s", message);
      break;
    case ::airplay::LogLevel::LOG_LEVEL_INFO:
      ESP_LOGI(TAG, "%s", message);
      break;
    case ::airplay::LogLevel::LOG_LEVEL_DEBUG:
      ESP_LOGD(TAG, "%s", message);
      break;
    default:
      ESP_LOGV(TAG, "%s", message);
      break;
  }
}

static void *allocate_buffer(size_t size) {
  // The jitter buffer goes to PSRAM when there is some; the receiver touches it only from its own task
  return heap_caps_malloc_prefer(size, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

void AirPlayMediaSource::setup() {
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  ::airplay::set_log_handler(library_log, ::airplay::LogLevel::LOG_LEVEL_VERBOSE);
#elif ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  ::airplay::set_log_handler(library_log, ::airplay::LogLevel::LOG_LEVEL_DEBUG);
#else
  ::airplay::set_log_handler(library_log, ::airplay::LogLevel::LOG_LEVEL_INFO);
#endif

  if (!this->crypto_.init()) {
    ESP_LOGE(TAG, "Failed to load the RSA key");
    this->mark_failed();
    return;
  }

  ::airplay::ReceiverConfig config;
  config.name = this->name_;
  get_mac_address_raw(config.mac);
  // Synchronized playback holds the sender's whole latency (about 2.25 s) in the buffer. Unsynchronized
  // playback fills three quarters of it, up to 500 ms, before it starts.
  config.buffer_ms = this->buffer_duration_ms_;
#ifdef USE_AIRPLAY_METADATA
  config.metadata = true;
#endif
  config.allocate = allocate_buffer;
  config.deallocate = heap_caps_free;
  this->receiver_ = std::make_unique<::airplay::Receiver>(config, this->crypto_, this->runner_, *this, this);

  this->lock_ = xSemaphoreCreateMutex();
  if (this->lock_ == nullptr || !this->receiver_->begin()) {
    ESP_LOGE(TAG, "Failed to allocate buffers");
    this->mark_failed();
    return;
  }

  // The AirPlay task comes last: once it runs, senders can connect
  if (!this->remote_task_.create(AirPlayMediaSource::remote_task, "airplay_remote", REMOTE_TASK_STACK_SIZE, this,
                                 REMOTE_TASK_PRIORITY, this->task_stack_in_psram_) ||
      !this->airplay_task_.create(AirPlayMediaSource::airplay_task, "airplay", AIRPLAY_TASK_STACK_SIZE, this,
                                  AIRPLAY_TASK_PRIORITY, this->task_stack_in_psram_)) {
    ESP_LOGE(TAG, "Failed to create tasks");
    this->mark_failed();
    return;
  }

  this->register_mdns_service_();
  this->disable_loop();
}

void AirPlayMediaSource::airplay_task(void *params) {
  auto *self = static_cast<AirPlayMediaSource *>(params);
  while (!self->runner_.begin(*self->receiver_, self->port_))
    vTaskDelay(pdMS_TO_TICKS(LISTEN_RETRY_MS));
  while (true)
    self->runner_.run_once(RUN_INTERVAL_MS);
}

void AirPlayMediaSource::register_mdns_service_() {
  char instance[64];
  this->receiver_->service_instance_name(instance, sizeof(instance));
  size_t count = 0;
  const ::airplay::TxtRecord *records = this->receiver_->txt_records(&count);
  mdns_txt_item_t txt[MAX_TXT_RECORDS];
  count = std::min(count, MAX_TXT_RECORDS);
  for (size_t i = 0; i < count; i++)
    txt[i] = mdns_txt_item_t{records[i].key, records[i].value};

  const esp_err_t err = mdns_service_add(instance, "_raop", "_tcp", this->port_, txt, count);
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
                "  Buffer duration: %" PRIu32 " ms\n"
                "  Task stack in PSRAM: %s\n"
                "  airplay-cpp: " AIRPLAY_CPP_VERSION,
                this->name_, this->port_, this->buffer_duration_ms_, YESNO(this->task_stack_in_psram_));
}

// THREAD CONTEXT: Main loop. Woken by the AirPlay task when the session changes.
void AirPlayMediaSource::loop() {
  const auto desired = static_cast<::airplay::SessionState>(this->session_state_.load(std::memory_order_acquire));
  const media_source::MediaSourceState state = this->get_state();

  switch (desired) {
    case ::airplay::SessionState::NONE:
      this->play_requested_ = false;
      if (state != media_source::MediaSourceState::IDLE)
        this->set_state_(media_source::MediaSourceState::IDLE);
      break;
    case ::airplay::SessionState::PLAYING:
    case ::airplay::SessionState::PAUSED:
      if (state == media_source::MediaSourceState::IDLE) {
        if (!this->play_requested_) {
          // Ask the orchestrator to switch to this source; it then calls play_uri()
          this->play_requested_ = true;
          this->request_play_uri_(PLAY_URI);
          // Ask again if the orchestrator dropped the request
          this->set_timeout(PLAY_RETRY_ID, PLAY_RETRY_MS, [this]() {
            this->play_requested_ = false;
            this->enable_loop();
          });
        }
      } else if (desired == ::airplay::SessionState::PLAYING && state == media_source::MediaSourceState::PAUSED) {
        this->set_state_(media_source::MediaSourceState::PLAYING);
      } else if (desired == ::airplay::SessionState::PAUSED && state == media_source::MediaSourceState::PLAYING) {
        this->set_state_(media_source::MediaSourceState::PAUSED);
      }
      break;
  }
  this->set_wifi_streaming_(desired != ::airplay::SessionState::NONE);
#ifdef USE_AIRPLAY_METADATA
  if (this->metadata_changed_.exchange(false, std::memory_order_acq_rel))
    this->publish_metadata_();
#endif
  this->disable_loop();
}

#ifdef USE_AIRPLAY_METADATA
// THREAD CONTEXT: Main loop
void AirPlayMediaSource::publish_metadata_() {
  ::airplay::Metadata metadata;
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  metadata = this->metadata_;
  xSemaphoreGive(this->lock_);
  if (this->title_text_sensor_ != nullptr)
    this->title_text_sensor_->publish_state(metadata.title);
  if (this->artist_text_sensor_ != nullptr)
    this->artist_text_sensor_->publish_state(metadata.artist);
  if (this->album_text_sensor_ != nullptr)
    this->album_text_sensor_->publish_state(metadata.album);
}
#endif

// THREAD CONTEXT: Main loop
void AirPlayMediaSource::set_wifi_streaming_(bool streaming) {
  if (streaming == this->wifi_streaming_)
    return;
  this->wifi_streaming_ = streaming;
#ifdef USE_WIFI
  // Power saving delays packets, and roaming scans take the radio off channel; both cause dropouts
  if (wifi::global_wifi_component == nullptr)
    return;
  if (streaming) {
#ifdef USE_WIFI_RUNTIME_POWER_SAVE
    wifi::global_wifi_component->request_high_performance();
#endif
#ifdef USE_WIFI_RUNTIME_ROAMING_SUPPRESSION
    wifi::global_wifi_component->request_roaming_suppression();
#endif
  } else {
#ifdef USE_WIFI_RUNTIME_POWER_SAVE
    wifi::global_wifi_component->release_high_performance();
#endif
#ifdef USE_WIFI_RUNTIME_ROAMING_SUPPRESSION
    wifi::global_wifi_component->release_roaming_suppression();
#endif
  }
#endif
}

// THREAD CONTEXT: AirPlay task
size_t AirPlayMediaSource::write(const int16_t *samples, size_t frames) {
  // Writes nothing unless this source is the active one
  const size_t written = this->write_output(reinterpret_cast<const uint8_t *>(samples),
                                            frames * ::airplay::BYTES_PER_FRAME, 0, this->stream_info_);
  return written / ::airplay::BYTES_PER_FRAME;
}

// THREAD CONTEXT: AirPlay task
void AirPlayMediaSource::on_session_state(::airplay::SessionState state) {
  if (state == ::airplay::SessionState::NONE) {
    this->sender_volume_.store(-1.0f, std::memory_order_relaxed);
    // Keep the remote control details for a while, so that a paused sender can be resumed from here
    xSemaphoreTake(this->lock_, portMAX_DELAY);
    this->remote_.session_active = false;
    this->remote_.session_end_ms = millis();
    xSemaphoreGive(this->lock_);
  }
  this->session_state_.store(static_cast<uint8_t>(state), std::memory_order_release);
  this->enable_loop_soon_any_context();
}

// THREAD CONTEXT: AirPlay task
void AirPlayMediaSource::on_volume(float volume) {
  this->sender_volume_.store(volume, std::memory_order_relaxed);
  this->request_volume_(volume);
}

// THREAD CONTEXT: AirPlay task
void AirPlayMediaSource::on_mute(bool muted) { this->request_mute_(muted); }

// THREAD CONTEXT: AirPlay task
void AirPlayMediaSource::on_metadata(const ::airplay::Metadata &metadata) {
#ifdef USE_AIRPLAY_METADATA
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  this->metadata_ = metadata;
  xSemaphoreGive(this->lock_);
  this->metadata_changed_.store(true, std::memory_order_release);
  this->enable_loop_soon_any_context();
#endif
}

// THREAD CONTEXT: AirPlay task. Remote control details arrive with the requests of the session owner.
void AirPlayMediaSource::on_remote_target(const ::airplay::RemoteTarget &target) {
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  RemoteState &remote = this->remote_;
  const bool changed =
      std::strcmp(remote.target.dacp_id, target.dacp_id) != 0 || remote.target.address != target.address;
  remote.target = target;
  if (changed)
    remote.port = 0;
  remote.session_active = true;
  xSemaphoreGive(this->lock_);
  // Look up the port now, so that the first command does not have to wait for it
  if (changed)
    this->send_remote_request_(REMOTE_REQUEST_DISCOVER);
}

bool AirPlayMediaSource::can_handle(const std::string &uri) const { return uri.starts_with(URI_PREFIX); }

// THREAD CONTEXT: Main loop
bool AirPlayMediaSource::play_uri(const std::string &uri) {
  this->play_requested_ = false;
  if (this->is_failed() || !this->can_handle(uri))
    return false;

  const auto desired = static_cast<::airplay::SessionState>(this->session_state_.load(std::memory_order_acquire));
  if (desired == ::airplay::SessionState::NONE) {
    // Accept and stay idle. Failing would make the orchestrator retry endlessly when repeat is enabled.
    ESP_LOGD(TAG, "No AirPlay sender is connected");
    return true;
  }
  // The orchestrator restarts its count of played frames right after this call
  this->receiver_->request_output_restart();
  // Report playing first even when paused: the orchestrator only makes a source active when it starts playing
  this->set_state_(media_source::MediaSourceState::PLAYING);
  if (desired == ::airplay::SessionState::PAUSED)
    this->set_state_(media_source::MediaSourceState::PAUSED);
  return true;
}

// THREAD CONTEXT: Main loop
void AirPlayMediaSource::handle_command(media_source::MediaSourceCommand command) {
  uint8_t request = 0;
  switch (command) {
    case media_source::MediaSourceCommand::STOP:
      if (this->session_state_.load(std::memory_order_acquire) == static_cast<uint8_t>(::airplay::SessionState::NONE)) {
        this->play_requested_ = false;
        this->set_state_(media_source::MediaSourceState::IDLE);
      } else {
        ESP_LOGI(TAG, "Disconnecting the sender");
        // Pause the sender too, so it does not switch to its own speaker and keep playing there
        this->send_remote_request_(REMOTE_REQUEST_PAUSE);
        this->receiver_->request_disconnect();
      }
      return;
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
  // Also works for a while after the sender disconnected: "play" then makes it reconnect
  this->send_remote_request_(request);
}

// THREAD CONTEXT: Main loop
void AirPlayMediaSource::notify_volume_changed(float volume) {
  // Move the sender's volume slider along, unless the change came from the sender
  float sender_volume = this->sender_volume_.load(std::memory_order_relaxed);
  if (sender_volume < 0.0f || std::fabs(volume - sender_volume) < VOLUME_CHANGE_THRESHOLD)
    return;
  // Fails if the session ended in the meantime
  if (!this->sender_volume_.compare_exchange_strong(sender_volume, volume, std::memory_order_relaxed))
    return;
  this->remote_volume_.store(volume, std::memory_order_relaxed);
  this->send_remote_request_(REMOTE_REQUEST_VOLUME);
}

// THREAD CONTEXT: Speaker playback callback
void AirPlayMediaSource::notify_audio_played(uint32_t frames, int64_t timestamp) {
  if (this->receiver_ != nullptr)
    this->receiver_->notify_played(frames, timestamp);
}

}  // namespace esphome::airplay

#endif  // USE_ESP32

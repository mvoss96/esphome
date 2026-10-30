#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif

#include <airplay.h>

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

static constexpr uint32_t DISCOVERY_TIMEOUT_MS = 2000;
static constexpr size_t DISCOVERY_MAX_RESULTS = 10;
static constexpr uint32_t COMMAND_TIMEOUT_MS = 1500;
/// Commands still reach a sender for this long after it disconnected, e.g. to resume after a pause
static constexpr uint32_t SESSION_GRACE_MS = 15 * 60 * 1000;

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

/// The receiver of the airplay-cpp library with its network runner, and the sender's remote control details.
/// The receiver calls write() and the listener methods from the AirPlay task.
class AirPlayBridge final : public ::airplay::AudioSink, public ::airplay::Listener {
 public:
  /// The sender's remote control (DACP) details, shared between the AirPlay and remote control tasks
  struct RemoteState {
    ::airplay::RemoteTarget target;
    uint16_t port;  // 0 until found through mDNS
    bool session_active;
    uint32_t session_end_ms;
  };

  explicit AirPlayBridge(AirPlayMediaSource *parent) : parent_(parent) {}

  bool begin() {
    if (!this->crypto.init()) {
      ESP_LOGE(TAG, "Failed to load the RSA key");
      return false;
    }
    ::airplay::ReceiverConfig config;
    config.name = this->parent_->name_;
    get_mac_address_raw(config.mac);
    // Synchronized playback holds the sender's whole latency (about 2.25 s) in the buffer. Unsynchronized
    // playback fills three quarters of it, up to 500 ms, before it starts.
    config.buffer_ms = this->parent_->buffer_duration_ms_;
#ifdef USE_AIRPLAY_METADATA
    config.metadata = true;
#endif
    config.allocate = allocate_buffer;
    config.deallocate = heap_caps_free;
    this->receiver = std::make_unique<::airplay::Receiver>(config, this->crypto, this->runner, *this, this);
    if (!this->receiver->begin()) {
      ESP_LOGE(TAG, "Failed to allocate buffers");
      return false;
    }
    return true;
  }

  // THREAD CONTEXT: AirPlay task
  size_t write(const int16_t *samples, size_t frames) override {
    // An output may take only part of a frame. The rest of that frame goes out before anything else, so that the
    // channels stay in place.
    if (this->partial_remaining_ > 0) {
      const size_t offset = ::airplay::BYTES_PER_FRAME - this->partial_remaining_;
      this->partial_remaining_ -=
          this->parent_->write_output(this->partial_frame_ + offset, this->partial_remaining_, 0, this->stream_info_);
      if (this->partial_remaining_ > 0)
        return 0;
    }
    // Writes nothing unless this source is the active one
    const auto *bytes = reinterpret_cast<const uint8_t *>(samples);
    const size_t written =
        this->parent_->write_output(bytes, frames * ::airplay::BYTES_PER_FRAME, 0, this->stream_info_);
    size_t whole = written / ::airplay::BYTES_PER_FRAME;
    const size_t part = written % ::airplay::BYTES_PER_FRAME;
    if (part > 0) {
      std::memcpy(this->partial_frame_, bytes + whole * ::airplay::BYTES_PER_FRAME, ::airplay::BYTES_PER_FRAME);
      this->partial_remaining_ = ::airplay::BYTES_PER_FRAME - part;
      whole++;  // Counted as written; its rest follows on the next call
    }
    return whole;
  }

  // THREAD CONTEXT: AirPlay task
  void on_session_state(::airplay::SessionState state) override {
    AirPlayMediaSource *parent = this->parent_;
    if (state == ::airplay::SessionState::NONE) {
      parent->sender_volume_.store(-1.0f, std::memory_order_relaxed);
      // Keep the remote control details for a while, so that a paused sender can be resumed from here
      xSemaphoreTake(parent->lock_, portMAX_DELAY);
      this->remote.session_active = false;
      this->remote.session_end_ms = millis();
      xSemaphoreGive(parent->lock_);
    }
    parent->session_state_.store(static_cast<uint8_t>(state), std::memory_order_release);
    parent->enable_loop_soon_any_context();
  }

  // THREAD CONTEXT: AirPlay task
  void on_volume(float volume) override {
    this->parent_->sender_volume_.store(volume, std::memory_order_relaxed);
    this->parent_->request_volume_(volume);
  }

  // THREAD CONTEXT: AirPlay task
  void on_mute(bool muted) override { this->parent_->request_mute_(muted); }

  // THREAD CONTEXT: AirPlay task
  void on_metadata(const ::airplay::Metadata &metadata) override {
#ifdef USE_AIRPLAY_METADATA
    AirPlayMediaSource *parent = this->parent_;
    xSemaphoreTake(parent->lock_, portMAX_DELAY);
    std::memcpy(parent->track_info_.title, metadata.title, sizeof(parent->track_info_.title));
    std::memcpy(parent->track_info_.artist, metadata.artist, sizeof(parent->track_info_.artist));
    std::memcpy(parent->track_info_.album, metadata.album, sizeof(parent->track_info_.album));
    xSemaphoreGive(parent->lock_);
    parent->metadata_changed_.store(true, std::memory_order_release);
    parent->enable_loop_soon_any_context();
#endif
  }

  // THREAD CONTEXT: AirPlay task. Remote control details arrive with the requests of the session owner.
  void on_remote_target(const ::airplay::RemoteTarget &target) override {
    AirPlayMediaSource *parent = this->parent_;
    xSemaphoreTake(parent->lock_, portMAX_DELAY);
    const bool changed =
        std::strcmp(this->remote.target.dacp_id, target.dacp_id) != 0 || this->remote.target.address != target.address;
    this->remote.target = target;
    if (changed)
      this->remote.port = 0;
    this->remote.session_active = true;
    xSemaphoreGive(parent->lock_);
    // Look up the port now, so that the first command does not have to wait for it
    if (changed)
      parent->send_remote_request_(AirPlayMediaSource::REMOTE_REQUEST_DISCOVER);
  }

  // THREAD CONTEXT: Remote control task. Network calls here may block for a few seconds without holding up
  // playback.
  void handle_remote_requests(uint8_t requests) {
    SemaphoreHandle_t lock = this->parent_->lock_;
    xSemaphoreTake(lock, portMAX_DELAY);
    RemoteState remote = this->remote;
    xSemaphoreGive(lock);

    const bool commands = (requests & ~AirPlayMediaSource::REMOTE_REQUEST_DISCOVER) != 0;
    if (remote.target.dacp_id[0] == '\0' || remote.target.active_remote[0] == '\0' ||
        (!remote.session_active && millis() - remote.session_end_ms > SESSION_GRACE_MS)) {
      if (commands) {
        ESP_LOGW(TAG, "No AirPlay sender with remote control support");
      }
      return;
    }

    if (remote.port == 0) {
      if (!this->discover_remote_port_(remote))
        return;
      xSemaphoreTake(lock, portMAX_DELAY);
      if (std::strcmp(this->remote.target.dacp_id, remote.target.dacp_id) == 0)
        this->remote.port = remote.port;
      xSemaphoreGive(lock);
    }

    struct Command {
      uint8_t request;
      const char *path;
    };
    static constexpr Command COMMANDS[] = {
        {AirPlayMediaSource::REMOTE_REQUEST_PLAY, ::airplay::DACP_PLAY},
        {AirPlayMediaSource::REMOTE_REQUEST_PAUSE, ::airplay::DACP_PAUSE},
        {AirPlayMediaSource::REMOTE_REQUEST_NEXT, ::airplay::DACP_NEXT},
        {AirPlayMediaSource::REMOTE_REQUEST_PREVIOUS, ::airplay::DACP_PREVIOUS},
    };
    bool unreachable = false;
    for (const auto &command : COMMANDS) {
      if ((requests & command.request) != 0)
        unreachable |= !this->send_command_(remote, command.path);
    }
    char volume_command[64];
    if ((requests & AirPlayMediaSource::REMOTE_REQUEST_VOLUME) != 0 &&
        ::airplay::dacp_volume_command(this->parent_->remote_volume_.load(std::memory_order_relaxed), volume_command,
                                       sizeof(volume_command)) > 0) {
      unreachable |= !this->send_command_(remote, volume_command);
    }

    if (unreachable) {
      // The port may have changed, so look it up again next time
      xSemaphoreTake(lock, portMAX_DELAY);
      if (std::strcmp(this->remote.target.dacp_id, remote.target.dacp_id) == 0)
        this->remote.port = 0;
      xSemaphoreGive(lock);
    }
  }

  ::airplay::MbedtlsCrypto crypto;
  ::airplay::SocketRunner runner;
  std::unique_ptr<::airplay::Receiver> receiver;
  RemoteState remote{};  // Guarded by the parent's lock_

 protected:
  /// @return false if the sender could not be reached
  bool send_command_(const RemoteState &remote, const char *command) {
    return ::airplay::send_dacp_command(remote.target, remote.port, command, COMMAND_TIMEOUT_MS) != 0;
  }

  bool discover_remote_port_(RemoteState &remote) {
    // The sender announces its remote control service as "iTunes_Ctrl_<DACP-ID>", or only "<DACP-ID>"
    mdns_result_t *results = nullptr;
    if (mdns_query_ptr(::airplay::DACP_SERVICE_TYPE, ::airplay::DACP_SERVICE_PROTOCOL, DISCOVERY_TIMEOUT_MS,
                       DISCOVERY_MAX_RESULTS, &results) == ESP_OK) {
      for (const mdns_result_t *result = results; result != nullptr; result = result->next) {
        if (result->instance_name != nullptr && result->port != 0 &&
            ::airplay::dacp_instance_matches(result->instance_name, remote.target.dacp_id)) {
          remote.port = result->port;
          break;
        }
      }
    }
    mdns_query_results_free(results);
    if (remote.port == 0) {
      ESP_LOGW(TAG, "Remote control service of the sender not found");
      return false;
    }
    ESP_LOGD(TAG, "Remote control service found on port %u", remote.port);
    return true;
  }

  AirPlayMediaSource *parent_;
  audio::AudioStreamInfo stream_info_{16, ::airplay::NUM_CHANNELS, ::airplay::SAMPLE_RATE};
  uint8_t partial_frame_[::airplay::BYTES_PER_FRAME]{};  // AirPlay task only
  size_t partial_remaining_{0};                          // Bytes of partial_frame_ not taken by the output yet
};

AirPlayMediaSource::AirPlayMediaSource(const char *name, uint16_t port, uint32_t buffer_duration_ms)
    : name_(name), buffer_duration_ms_(buffer_duration_ms), port_(port) {}

AirPlayMediaSource::~AirPlayMediaSource() = default;

void AirPlayMediaSource::setup() {
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  ::airplay::set_log_handler(library_log, ::airplay::LogLevel::LOG_LEVEL_VERBOSE);
#elif ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  ::airplay::set_log_handler(library_log, ::airplay::LogLevel::LOG_LEVEL_DEBUG);
#else
  ::airplay::set_log_handler(library_log, ::airplay::LogLevel::LOG_LEVEL_INFO);
#endif

  this->lock_ = xSemaphoreCreateMutex();
  this->bridge_ = std::make_unique<AirPlayBridge>(this);
  if (this->lock_ == nullptr || !this->bridge_->begin()) {
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

// THREAD CONTEXT: AirPlay task
void AirPlayMediaSource::airplay_task(void *params) {
  auto *self = static_cast<AirPlayMediaSource *>(params);
  AirPlayBridge &bridge = *self->bridge_;
  while (!bridge.runner.begin(*bridge.receiver, self->port_))
    vTaskDelay(pdMS_TO_TICKS(LISTEN_RETRY_MS));
  while (true)
    bridge.runner.run_once(RUN_INTERVAL_MS);
}

// THREAD CONTEXT: Remote control task
void AirPlayMediaSource::remote_task(void *params) {
  auto *self = static_cast<AirPlayMediaSource *>(params);
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint8_t requests = self->remote_requests_.exchange(0, std::memory_order_acq_rel);
    if (requests != 0)
      self->bridge_->handle_remote_requests(requests);
  }
}

// THREAD CONTEXT: Any. Wakes the remote control task.
void AirPlayMediaSource::send_remote_request_(uint8_t request) {
  this->remote_requests_.fetch_or(request, std::memory_order_acq_rel);
  if (this->remote_task_.is_created())
    xTaskNotifyGive(this->remote_task_.get_handle());
}

void AirPlayMediaSource::register_mdns_service_() {
  const ::airplay::Receiver &receiver = *this->bridge_->receiver;
  char instance[64];
  receiver.service_instance_name(instance, sizeof(instance));
  size_t count = 0;
  const ::airplay::TxtRecord *records = receiver.txt_records(&count);
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
          // Ask again if the orchestrator dropped the request
          this->set_timeout(PLAY_RETRY_ID, PLAY_RETRY_MS, [this]() {
            this->play_requested_ = false;
            this->enable_loop();
          });
        }
      } else if (desired == SESSION_STATE_PLAYING && state == media_source::MediaSourceState::PAUSED) {
        this->set_state_(media_source::MediaSourceState::PLAYING);
      } else if (desired == SESSION_STATE_PAUSED && state == media_source::MediaSourceState::PLAYING) {
        this->set_state_(media_source::MediaSourceState::PAUSED);
      }
      break;
  }
  this->set_wifi_streaming_(desired != SESSION_STATE_NONE);
#ifdef USE_AIRPLAY_METADATA
  if (this->metadata_changed_.exchange(false, std::memory_order_acq_rel))
    this->publish_metadata_();
#endif
  this->disable_loop();
}

#ifdef USE_AIRPLAY_METADATA
// THREAD CONTEXT: Main loop
void AirPlayMediaSource::publish_metadata_() {
  TrackInfo info;
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  info = this->track_info_;
  xSemaphoreGive(this->lock_);
  if (this->title_text_sensor_ != nullptr)
    this->title_text_sensor_->publish_state(info.title);
  if (this->artist_text_sensor_ != nullptr)
    this->artist_text_sensor_->publish_state(info.artist);
  if (this->album_text_sensor_ != nullptr)
    this->album_text_sensor_->publish_state(info.album);
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

bool AirPlayMediaSource::can_handle(const std::string &uri) const { return uri.starts_with(URI_PREFIX); }

// THREAD CONTEXT: Main loop
bool AirPlayMediaSource::play_uri(const std::string &uri) {
  this->play_requested_ = false;
  if (this->is_failed() || !this->can_handle(uri))
    return false;

  const auto desired = static_cast<SessionState>(this->session_state_.load(std::memory_order_acquire));
  if (desired == SESSION_STATE_NONE) {
    // Accept and stay idle. Failing would make the orchestrator retry endlessly when repeat is enabled.
    ESP_LOGD(TAG, "No AirPlay sender is connected");
    return true;
  }
  // The orchestrator restarts its count of played frames right after this call
  this->bridge_->receiver->request_output_restart();
  // Report playing first even when paused: the orchestrator only makes a source active when it starts playing
  this->set_state_(media_source::MediaSourceState::PLAYING);
  if (desired == SESSION_STATE_PAUSED)
    this->set_state_(media_source::MediaSourceState::PAUSED);
  return true;
}

// THREAD CONTEXT: Main loop
void AirPlayMediaSource::handle_command(media_source::MediaSourceCommand command) {
  uint8_t request = 0;
  switch (command) {
    case media_source::MediaSourceCommand::STOP:
      if (this->session_state_.load(std::memory_order_acquire) == SESSION_STATE_NONE) {
        this->play_requested_ = false;
        this->set_state_(media_source::MediaSourceState::IDLE);
      } else if (!this->is_failed()) {
        ESP_LOGI(TAG, "Disconnecting the sender");
        // Pause the sender too, so it does not switch to its own speaker and keep playing there
        this->send_remote_request_(REMOTE_REQUEST_PAUSE);
        this->bridge_->receiver->request_disconnect();
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
  if (this->bridge_ != nullptr && this->bridge_->receiver != nullptr)
    this->bridge_->receiver->notify_played(frames, timestamp);
}

}  // namespace esphome::airplay

#endif  // USE_ESP32

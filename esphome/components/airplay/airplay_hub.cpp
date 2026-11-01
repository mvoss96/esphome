#include "airplay_hub.h"

#ifdef USE_ESP32

#include "media_source/airplay_media_source.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_AIRPLAY_ARTWORK
#include "image/airplay_image.h"
#endif

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif

#include <esp_heap_caps.h>
#include <freertos/task.h>
#include <mdns.h>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstring>

namespace esphome::airplay_ {

ESPHOME_LOG_TAG(TAG, "airplay");

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
/// How long the sender may take to report a volume change made from here
static constexpr uint32_t ABSOLUTE_VOLUME_WAIT_MS = 600;
static constexpr uint32_t VOLUME_STEP_WAIT_MS = 400;
static constexpr uint32_t VOLUME_POLL_MS = 20;
/// Limit for one adjustment, in case the sender's reports do not add up
static constexpr int MAX_VOLUME_STEPS = 24;
/// Commands still reach a sender for this long after it disconnected, e.g. to resume after a pause
static constexpr uint32_t SESSION_GRACE_MS = 15 * 60 * 1000;

static constexpr uint32_t MDNS_RETRY_ID = 0;
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

AirPlayHub::AirPlayHub(const char *name, uint16_t port) : name_(name), port_(port) {}

AirPlayHub::~AirPlayHub() = default;

void AirPlayHub::setup() {
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  ::airplay::set_log_handler(library_log, ::airplay::LogLevel::LOG_LEVEL_VERBOSE);
#elif ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  ::airplay::set_log_handler(library_log, ::airplay::LogLevel::LOG_LEVEL_DEBUG);
#else
  ::airplay::set_log_handler(library_log, ::airplay::LogLevel::LOG_LEVEL_INFO);
#endif

  this->lock_ = xSemaphoreCreateMutex();
  if (this->lock_ == nullptr || !this->crypto_.init()) {
    ESP_LOGE(TAG, "Failed to load the RSA key");
    this->mark_failed();
    return;
  }
  ::airplay::ReceiverConfig config;
  config.name = this->name_;
  config.model = this->model_;
  config.password = this->password_;
  get_mac_address_raw(config.mac);
  // Synchronized playback holds the sender's whole latency (about 2.25 s) in the buffer. Unsynchronized playback
  // fills three quarters of it, up to 500 ms, before it starts.
  config.buffer_ms = this->buffer_duration_ms_;
#ifdef USE_AIRPLAY_METADATA
  config.metadata = true;
#endif
#ifdef USE_AIRPLAY_ARTWORK
  config.artwork = this->artwork_image_ != nullptr;
#endif
#ifdef USE_AIRPLAY_PROGRESS
  config.progress = true;
#endif
  config.airplay2 = this->airplay2_;
  config.airplay2_features = this->airplay2_features_;
#ifdef USE_AIRPLAY_AAC
  config.aac_decoder = &this->aac_decoder_;
  config.buffered_audio_bytes = this->aac_buffer_size_;
#endif
  config.random = [](uint8_t *buffer, size_t length) { random_bytes(buffer, length); };
  config.allocate = allocate_buffer;
  config.deallocate = heap_caps_free;
  this->receiver_ = std::make_unique<::airplay::Receiver>(config, this->crypto_, this->runner_, *this, this);
  if (!this->receiver_->begin()) {
    ESP_LOGE(TAG, "Failed to allocate buffers");
    this->mark_failed();
    return;
  }

  // The AirPlay task comes last: once it runs, senders can connect
  if (!this->remote_task_.create(AirPlayHub::remote_task, "airplay_remote", REMOTE_TASK_STACK_SIZE, this,
                                 REMOTE_TASK_PRIORITY, this->task_stack_in_psram_) ||
      !this->airplay_task_.create(AirPlayHub::airplay_task, "airplay", AIRPLAY_TASK_STACK_SIZE, this,
                                  AIRPLAY_TASK_PRIORITY, this->task_stack_in_psram_)) {
    ESP_LOGE(TAG, "Failed to create tasks");
    this->mark_failed();
    return;
  }

  this->register_mdns_service_();
  this->disable_loop();
}

void AirPlayHub::dump_config() {
  ESP_LOGCONFIG(TAG,
                "AirPlay:\n"
                "  Name: %s\n"
                "  Model: %s\n"
                "  Port: %u\n"
                "  Password: %s\n"
                "  AirPlay 2: %s\n"
                "  Buffer duration: %" PRIu32 " ms\n"
                "  Task stack in PSRAM: %s\n"
                "  airplay-cpp: " AIRPLAY_CPP_VERSION,
                this->name_, this->model_, this->port_, YESNO(this->password_ != nullptr),
                YESNO(this->receiver_ != nullptr && this->receiver_->airplay2_enabled()), this->buffer_duration_ms_,
                YESNO(this->task_stack_in_psram_));
}

// THREAD CONTEXT: AirPlay task
void AirPlayHub::airplay_task(void *params) {
  auto *self = static_cast<AirPlayHub *>(params);
  while (!self->runner_.begin(*self->receiver_, self->port_))
    vTaskDelay(pdMS_TO_TICKS(LISTEN_RETRY_MS));
  while (true)
    self->runner_.run_once(RUN_INTERVAL_MS);
}

// THREAD CONTEXT: Remote control task
void AirPlayHub::remote_task(void *params) {
  auto *self = static_cast<AirPlayHub *>(params);
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint8_t requests = self->remote_requests_.exchange(0, std::memory_order_acq_rel);
    if (requests != 0)
      self->handle_remote_requests_(requests);
  }
}

/// Adds an mDNS service unless it is already registered or has no TXT records
static bool add_mdns_service(const char *instance, const char *type, uint16_t port, const ::airplay::TxtRecord *records,
                             size_t count) {
  if (count == 0 || mdns_service_exists(type, "_tcp", nullptr))
    return true;
  mdns_txt_item_t txt[MAX_TXT_RECORDS];
  count = std::min(count, MAX_TXT_RECORDS);
  for (size_t i = 0; i < count; i++)
    txt[i] = mdns_txt_item_t{records[i].key, records[i].value};
  const esp_err_t err = mdns_service_add(instance, type, "_tcp", port, txt, count);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to register the mDNS service %s, retrying: %s", type, esp_err_to_name(err));
    return false;
  }
  ESP_LOGD(TAG, "Registered mDNS service %s.%s._tcp", instance, type);
  return true;
}

void AirPlayHub::register_mdns_service_() {
  char instance[64];
  this->receiver_->service_instance_name(instance, sizeof(instance));
  size_t raop_count = 0;
  const ::airplay::TxtRecord *raop = this->receiver_->txt_records(&raop_count);
  // AirPlay 2 senders look for _airplay._tcp, whose instance name is the plain name; none without AirPlay 2
  size_t airplay_count = 0;
  const ::airplay::TxtRecord *airplay = this->receiver_->airplay_txt_records(&airplay_count);
  if (!add_mdns_service(instance, "_raop", this->port_, raop, raop_count) ||
      !add_mdns_service(this->name_, "_airplay", this->port_, airplay, airplay_count)) {
    this->set_timeout(MDNS_RETRY_ID, MDNS_RETRY_INTERVAL_MS, [this]() { this->register_mdns_service_(); });
  }
}

// THREAD CONTEXT: Main loop. Woken by the AirPlay task.
void AirPlayHub::loop() {
  if (this->session_changed_.exchange(false, std::memory_order_acq_rel)) {
    this->set_wifi_streaming_(this->get_session_state() != ::airplay::SessionState::NONE);
#ifdef USE_AIRPLAY_PROGRESS
    this->update_progress_clock_();
#endif
  }
#ifdef USE_AIRPLAY_PROGRESS
  if (this->progress_changed_.exchange(false, std::memory_order_acq_rel)) {
    xSemaphoreTake(this->lock_, portMAX_DELAY);
    const ::airplay::Progress progress = this->progress_;
    xSemaphoreGive(this->lock_);
    this->progress_duration_ms_ = progress.duration_ms;
    this->progress_base_ms_ = progress.position_ms;
    this->progress_since_ms_ = App.get_loop_component_start_time();
    this->progress_running_ = false;
    this->update_progress_clock_();
  }
#endif
#ifdef USE_AIRPLAY_METADATA
  if (this->metadata_changed_.exchange(false, std::memory_order_acq_rel)) {
    ::airplay::Metadata metadata;
    xSemaphoreTake(this->lock_, portMAX_DELAY);
    metadata = this->metadata_;
    xSemaphoreGive(this->lock_);
    this->metadata_callback_.call(metadata);
  }
#endif
  this->disable_loop();
}

#ifdef USE_AIRPLAY_PROGRESS
// THREAD CONTEXT: Main loop. Starts or stops the position clock as the session plays or pauses.
void AirPlayHub::update_progress_clock_() {
  const uint32_t now = App.get_loop_component_start_time();
  const bool running = this->progress_duration_ms_ > 0 && this->get_session_state() == ::airplay::SessionState::PLAYING;
  if (running != this->progress_running_) {
    if (this->progress_running_)
      this->progress_base_ms_ += now - this->progress_since_ms_;
    this->progress_since_ms_ = now;
    this->progress_running_ = running;
  }
  this->progress_callback_.call();
}

uint32_t AirPlayHub::get_track_progress_ms() const {
  uint32_t position = this->progress_base_ms_;
  if (this->progress_running_)
    position += App.get_loop_component_start_time() - this->progress_since_ms_;
  return std::min(position, this->progress_duration_ms_);
}
#endif

// THREAD CONTEXT: Main loop
void AirPlayHub::set_wifi_streaming_(bool streaming) {
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

// ===== For the media source =====

void AirPlayHub::request_output_restart() {
  if (this->receiver_ != nullptr)
    this->receiver_->request_output_restart();
}

void AirPlayHub::request_disconnect() {
  if (this->receiver_ == nullptr)
    return;
  ESP_LOGI(TAG, "Disconnecting the sender");
  // Pause the sender too, so it does not switch to its own speaker and keep playing there
  this->send_remote_request(REMOTE_REQUEST_PAUSE);
  this->receiver_->request_disconnect();
}

// THREAD CONTEXT: Any. Wakes the remote control task.
void AirPlayHub::send_remote_request(uint8_t request) {
  this->remote_requests_.fetch_or(request, std::memory_order_acq_rel);
  if (this->remote_task_.is_created())
    xTaskNotifyGive(this->remote_task_.get_handle());
}

void AirPlayHub::set_speaker_volume(float volume) {
  float sender_volume = this->sender_volume_.load(std::memory_order_relaxed);
  if (sender_volume < 0.0f || std::fabs(volume - sender_volume) < VOLUME_CHANGE_THRESHOLD)
    return;
  // Fails if the session ended in the meantime
  if (!this->sender_volume_.compare_exchange_strong(sender_volume, volume, std::memory_order_relaxed))
    return;
  this->remote_volume_.store(volume, std::memory_order_relaxed);
  this->send_remote_request(REMOTE_REQUEST_VOLUME);
}

// THREAD CONTEXT: Speaker playback callback
void AirPlayHub::notify_played(uint32_t frames, int64_t timestamp) {
  if (this->receiver_ != nullptr)
    this->receiver_->notify_played(frames, timestamp);
}

// ===== AirPlay task =====

size_t AirPlayHub::write(const int16_t *samples, size_t frames) {
  if (this->media_source_ == nullptr)
    return frames;
  // An output may take only part of a frame. The rest of that frame goes out before anything else, so that the
  // channels stay in place.
  if (this->partial_remaining_ > 0) {
    const size_t offset = ::airplay::BYTES_PER_FRAME - this->partial_remaining_;
    this->partial_remaining_ -=
        this->media_source_->write_audio(this->partial_frame_ + offset, this->partial_remaining_);
    if (this->partial_remaining_ > 0)
      return 0;
  }
  // Writes nothing unless the media source is the active one
  const auto *bytes = reinterpret_cast<const uint8_t *>(samples);
  const size_t written = this->media_source_->write_audio(bytes, frames * ::airplay::BYTES_PER_FRAME);
  size_t whole = written / ::airplay::BYTES_PER_FRAME;
  const size_t part = written % ::airplay::BYTES_PER_FRAME;
  if (part > 0) {
    std::memcpy(this->partial_frame_, bytes + whole * ::airplay::BYTES_PER_FRAME, ::airplay::BYTES_PER_FRAME);
    this->partial_remaining_ = ::airplay::BYTES_PER_FRAME - part;
    whole++;  // Counted as written; its rest follows on the next call
  }
  return whole;
}

void AirPlayHub::on_session_state(::airplay::SessionState state) {
  if (state == ::airplay::SessionState::NONE) {
    this->sender_volume_.store(-1.0f, std::memory_order_relaxed);
    this->reported_volume_.store(-1.0f, std::memory_order_relaxed);
    // Keep the remote control details for a while, so that a paused sender can be resumed from here
    xSemaphoreTake(this->lock_, portMAX_DELAY);
    this->remote_.session_active = false;
    this->remote_.session_end_ms = millis();
    xSemaphoreGive(this->lock_);
  }
  this->session_state_.store(static_cast<uint8_t>(state), std::memory_order_release);
  this->session_changed_.store(true, std::memory_order_release);
  this->enable_loop_soon_any_context();
  if (this->media_source_ != nullptr)
    this->media_source_->on_session_changed();
}

void AirPlayHub::on_volume(float volume) {
  this->reported_volume_.store(volume, std::memory_order_relaxed);
  // While the volume is moved from here, the sender reports each step; the target is already set here
  if (this->volume_adjusting_.load(std::memory_order_relaxed))
    return;
  this->sender_volume_.store(volume, std::memory_order_relaxed);
  if (this->media_source_ != nullptr)
    this->media_source_->on_sender_volume(volume);
}

void AirPlayHub::on_mute(bool muted) {
  // The sender's slider is at the bottom: volume steps down cannot move it any further
  float reported = this->reported_volume_.load(std::memory_order_relaxed);
  if (muted && reported > 0.0f)
    this->reported_volume_.compare_exchange_strong(reported, 0.0f, std::memory_order_relaxed);
  if (this->media_source_ != nullptr)
    this->media_source_->on_sender_mute(muted);
}

void AirPlayHub::on_metadata(const ::airplay::Metadata &metadata) {
#ifdef USE_AIRPLAY_METADATA
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  this->metadata_ = metadata;
  xSemaphoreGive(this->lock_);
  this->metadata_changed_.store(true, std::memory_order_release);
  this->enable_loop_soon_any_context();
#endif
}

void AirPlayHub::on_progress(const ::airplay::Progress &progress) {
#ifdef USE_AIRPLAY_PROGRESS
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  this->progress_ = progress;
  xSemaphoreGive(this->lock_);
  this->progress_changed_.store(true, std::memory_order_release);
  this->enable_loop_soon_any_context();
#endif
}

// Remote control details arrive with the requests of the session owner
void AirPlayHub::on_remote_target(const ::airplay::RemoteTarget &target) {
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  const bool changed =
      std::strcmp(this->remote_.target.dacp_id, target.dacp_id) != 0 || this->remote_.target.address != target.address;
  this->remote_.target = target;
  if (changed)
    this->remote_.port = 0;
  this->remote_.session_active = true;
  xSemaphoreGive(this->lock_);
  // Look up the port now, so that the first command does not have to wait for it
  if (changed)
    this->send_remote_request(REMOTE_REQUEST_DISCOVER);
}

#ifdef USE_AIRPLAY_ARTWORK
bool AirPlayHub::on_artwork_begin(::airplay::ArtworkFormat format, size_t size) {
  return this->artwork_image_->artwork_begin(format == ::airplay::ArtworkFormat::PNG, size);
}
void AirPlayHub::on_artwork_data(const uint8_t *data, size_t length) {
  this->artwork_image_->artwork_data(data, length);
}
void AirPlayHub::on_artwork_end(bool complete) { this->artwork_image_->artwork_end(complete); }
void AirPlayHub::on_artwork_clear() { this->artwork_image_->artwork_clear(); }
#endif

// ===== Remote control task. Network calls here may block for a few seconds without holding up playback. =====

void AirPlayHub::handle_remote_requests_(uint8_t requests) {
  xSemaphoreTake(this->lock_, portMAX_DELAY);
  RemoteState remote = this->remote_;
  xSemaphoreGive(this->lock_);

  const bool commands = (requests & ~REMOTE_REQUEST_DISCOVER) != 0;
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
    xSemaphoreTake(this->lock_, portMAX_DELAY);
    if (std::strcmp(this->remote_.target.dacp_id, remote.target.dacp_id) == 0)
      this->remote_.port = remote.port;
    xSemaphoreGive(this->lock_);
  }

  struct Command {
    uint8_t request;
    const char *path;
  };
  static constexpr Command COMMANDS[] = {
      {REMOTE_REQUEST_PLAY, ::airplay::DACP_PLAY},
      {REMOTE_REQUEST_PAUSE, ::airplay::DACP_PAUSE},
      {REMOTE_REQUEST_NEXT, ::airplay::DACP_NEXT},
      {REMOTE_REQUEST_PREVIOUS, ::airplay::DACP_PREVIOUS},
  };
  bool unreachable = false;
  for (const auto &command : COMMANDS) {
    if ((requests & command.request) != 0)
      unreachable |= !this->send_command_(remote, command.path);
  }
  if ((requests & REMOTE_REQUEST_VOLUME) != 0)
    unreachable |= !this->adjust_volume_(remote);

  if (unreachable) {
    // The port may have changed, so look it up again next time
    xSemaphoreTake(this->lock_, portMAX_DELAY);
    if (std::strcmp(this->remote_.target.dacp_id, remote.target.dacp_id) == 0)
      this->remote_.port = 0;
    xSemaphoreGive(this->lock_);
  }
}

bool AirPlayHub::send_command_(const RemoteState &remote, const char *command) {
  return ::airplay::send_dacp_command(remote.target, remote.port, command, COMMAND_TIMEOUT_MS) != 0;
}

/// Moves the sender's volume slider to remote_volume_. @return false if the sender could not be reached
bool AirPlayHub::adjust_volume_(const RemoteState &remote) {
  if (std::strcmp(this->volume_dacp_id_, remote.target.dacp_id) != 0) {
    std::memcpy(this->volume_dacp_id_, remote.target.dacp_id, sizeof(this->volume_dacp_id_));
    this->absolute_volume_ = ABSOLUTE_VOLUME_UNKNOWN;
  }
  const bool session = this->reported_volume_.load(std::memory_order_relaxed) >= 0.0f;
  this->volume_adjusting_.store(true, std::memory_order_relaxed);
  bool reachable = true;
  if (this->absolute_volume_ != ABSOLUTE_VOLUME_UNSUPPORTED || !session) {
    const float target = this->remote_volume_.load(std::memory_order_relaxed);
    char command[64];
    if (::airplay::dacp_volume_command(target, command, sizeof(command)) > 0) {
      reachable = this->send_command_(remote, command);
      // Without a session there is no report to wait for
      if (reachable && session) {
        if (this->wait_for_volume_(target, ABSOLUTE_VOLUME_WAIT_MS)) {
          this->absolute_volume_ = ABSOLUTE_VOLUME_SUPPORTED;
        } else if (this->absolute_volume_ == ABSOLUTE_VOLUME_UNKNOWN) {
          ESP_LOGD(TAG, "The sender ignores absolute volume commands, using volume steps");
          this->absolute_volume_ = ABSOLUTE_VOLUME_UNSUPPORTED;
        }
      }
    }
  }
  if (reachable && session && this->absolute_volume_ == ABSOLUTE_VOLUME_UNSUPPORTED)
    reachable = this->step_volume_(remote);
  this->volume_adjusting_.store(false, std::memory_order_relaxed);
  return reachable;
}

/// Moves the volume with up and down commands, following the sender's reports
bool AirPlayHub::step_volume_(const RemoteState &remote) {
  float position = this->reported_volume_.load(std::memory_order_relaxed);
  for (int i = 0; i < MAX_VOLUME_STEPS && position >= 0.0f; i++) {
    // Read again on every step: the volume may still be changing here
    const int steps = ::airplay::dacp_volume_steps(position, this->remote_volume_.load(std::memory_order_relaxed));
    if (steps == 0)
      break;
    const float before = this->reported_volume_.load(std::memory_order_relaxed);
    if (!this->send_command_(remote, steps > 0 ? ::airplay::DACP_VOLUME_UP : ::airplay::DACP_VOLUME_DOWN))
      return false;
    if (this->wait_for_report_(before, VOLUME_STEP_WAIT_MS)) {
      position = this->reported_volume_.load(std::memory_order_relaxed);
    } else {
      // No report: assume the step was taken, also for the next adjustment
      const float step = steps > 0 ? ::airplay::DACP_VOLUME_STEP : -::airplay::DACP_VOLUME_STEP;
      position = std::clamp(position + step, 0.0f, 1.0f);
      float expected = before;
      this->reported_volume_.compare_exchange_strong(expected, position, std::memory_order_relaxed);
    }
  }
  return true;
}

/// @return true once the sender reports a volume within half a step of `target`
bool AirPlayHub::wait_for_volume_(float target, uint32_t timeout_ms) {
  const uint32_t start = millis();
  do {
    const float reported = this->reported_volume_.load(std::memory_order_relaxed);
    if (std::fabs(reported - target) < ::airplay::DACP_VOLUME_STEP / 2)
      return true;
    vTaskDelay(pdMS_TO_TICKS(VOLUME_POLL_MS));
  } while (millis() - start < timeout_ms);
  return false;
}

/// @return true once the sender reports a volume other than `before`
bool AirPlayHub::wait_for_report_(float before, uint32_t timeout_ms) {
  const uint32_t start = millis();
  do {
    if (this->reported_volume_.load(std::memory_order_relaxed) != before)
      return true;
    vTaskDelay(pdMS_TO_TICKS(VOLUME_POLL_MS));
  } while (millis() - start < timeout_ms);
  return false;
}

bool AirPlayHub::discover_remote_port_(RemoteState &remote) {
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

}  // namespace esphome::airplay_

#endif  // USE_ESP32

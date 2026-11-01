#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/static_task.h"

#include <airplay.h>

#include "aac_decoder.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <atomic>
#include <cstdint>
#include <memory>

// The namespace has a trailing underscore so that it does not clash with the airplay-cpp library's `airplay`
// namespace in the generated main.cpp, which uses `using namespace esphome`.
namespace esphome::airplay_ {

namespace airplay_priority {
// Registers its mDNS service, so it must run after the mdns component
inline constexpr float HUB = setup_priority::LATE;
inline constexpr float CHILD = HUB - 1.0f;
}  // namespace airplay_priority

class AirPlayMediaSource;
#ifdef USE_AIRPLAY_ARTWORK
class AirPlayImage;
#endif

/// AirPlay 1 (RAOP) and, optionally, AirPlay 2 receiver. The protocol is handled by the airplay-cpp library; the hub
/// runs it on its own task, announces it through mDNS and controls the sender. The media source, text sensors and cover
/// art image are its children.
///
/// THREADING: the library calls write() and the Listener methods on the AirPlay task. Remote control commands to the
/// sender run on a task of their own, because they may block for a few seconds. Children are notified on the main
/// loop, except for audio and the cover art bytes.
class AirPlayHub final : public Component, public ::airplay::AudioSink, public ::airplay::Listener {
 public:
  /// Requests to the remote control task, sent as bits of an atomic
  enum RemoteRequest : uint8_t {
    REMOTE_REQUEST_DISCOVER = 1 << 0,
    REMOTE_REQUEST_PLAY = 1 << 1,
    REMOTE_REQUEST_PAUSE = 1 << 2,
    REMOTE_REQUEST_NEXT = 1 << 3,
    REMOTE_REQUEST_PREVIOUS = 1 << 4,
    REMOTE_REQUEST_VOLUME = 1 << 5,
  };

  AirPlayHub(const char *name, uint16_t port);
  ~AirPlayHub() override;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return airplay_priority::HUB; }

  void set_task_stack_in_psram(bool task_stack_in_psram) { this->task_stack_in_psram_ = task_stack_in_psram; }
  void set_model(const char *model) { this->model_ = model; }
  void set_password(const char *password) { this->password_ = password; }
  void set_airplay2(bool airplay2) { this->airplay2_ = airplay2; }
  void set_airplay2_features(uint64_t features) { this->airplay2_features_ = features; }
#ifdef USE_AIRPLAY_AAC
  void set_aac_buffer_size(uint32_t size) { this->aac_buffer_size_ = size; }
#endif
  void set_buffer_duration(uint32_t buffer_duration_ms) { this->buffer_duration_ms_ = buffer_duration_ms; }
  void set_media_source(AirPlayMediaSource *media_source) { this->media_source_ = media_source; }
#ifdef USE_AIRPLAY_ARTWORK
  void set_artwork_image(AirPlayImage *image) { this->artwork_image_ = image; }
#endif
#ifdef USE_AIRPLAY_PROGRESS
  /// Called on the main loop when the track position jumps, the track changes or playback starts or stops
  template<typename F> void add_progress_callback(F &&callback) {
    this->progress_callback_.add(std::forward<F>(callback));
  }
  /// Length of the current track; 0 when unknown or no track plays
  uint32_t get_track_duration_ms() const { return this->progress_duration_ms_; }
  /// Position in the current track, advancing while it plays
  uint32_t get_track_progress_ms() const;
  bool is_progress_running() const { return this->progress_running_; }
#endif
#ifdef USE_AIRPLAY_METADATA
  /// Called on the main loop with the details of a new track; empty fields when the session ends
  template<typename F> void add_metadata_callback(F &&callback) {
    this->metadata_callback_.add(std::forward<F>(callback));
  }
#endif

  // ===== For the media source; main loop unless noted =====
  /// State of the AirPlay session. THREAD CONTEXT: any
  ::airplay::SessionState get_session_state() const {
    return static_cast<::airplay::SessionState>(this->session_state_.load(std::memory_order_acquire));
  }
  /// The output pipeline was restarted and holds none of the audio written before
  void request_output_restart();
  /// Ends the session, e.g. when another source takes over the speaker. Pauses the sender first.
  void request_disconnect();
  void send_remote_request(uint8_t request);
  /// The speaker volume changed; moves the sender's volume slider along unless the change came from the sender
  void set_speaker_volume(float volume);
  /// THREAD CONTEXT: speaker playback callback
  void notify_played(uint32_t frames, int64_t timestamp);

  // ===== airplay::AudioSink and airplay::Listener, on the AirPlay task =====
  size_t write(const int16_t *samples, size_t frames) override;
  void on_session_state(::airplay::SessionState state) override;
  void on_volume(float volume) override;
  void on_mute(bool muted) override;
  void on_metadata(const ::airplay::Metadata &metadata) override;
  void on_progress(const ::airplay::Progress &progress) override;
  void on_remote_target(const ::airplay::RemoteTarget &target) override;
#ifdef USE_AIRPLAY_ARTWORK
  bool on_artwork_begin(::airplay::ArtworkFormat format, size_t size) override;
  void on_artwork_data(const uint8_t *data, size_t length) override;
  void on_artwork_end(bool complete) override;
  void on_artwork_clear() override;
#endif

 protected:
  /// The sender's remote control (DACP) details, shared between the AirPlay and remote control tasks
  struct RemoteState {
    ::airplay::RemoteTarget target;
    uint16_t port;  // 0 until found through mDNS
    bool session_active;
    uint32_t session_end_ms;
  };

  /// Whether the sender takes an absolute volume (iTunes and Music on a Mac) or only steps (iPhone and iPad)
  enum AbsoluteVolume : uint8_t {
    ABSOLUTE_VOLUME_UNKNOWN,
    ABSOLUTE_VOLUME_SUPPORTED,
    ABSOLUTE_VOLUME_UNSUPPORTED,
  };

  static void airplay_task(void *params);
  static void remote_task(void *params);
  void register_mdns_service_();
  void set_wifi_streaming_(bool streaming);

  // Remote control task
  void handle_remote_requests_(uint8_t requests);
  /// @return false if the sender could not be reached
  bool send_command_(const RemoteState &remote, const char *command);
  bool adjust_volume_(const RemoteState &remote);
  bool step_volume_(const RemoteState &remote);
  bool wait_for_volume_(float target, uint32_t timeout_ms);
  bool wait_for_report_(float before, uint32_t timeout_ms);
  bool discover_remote_port_(RemoteState &remote);

  // Configuration
  const char *name_;
  const char *model_{"ESPHome"};
  const char *password_{nullptr};
  uint32_t buffer_duration_ms_{500};
  uint16_t port_;
  uint64_t airplay2_features_{0};
#ifdef USE_AIRPLAY_AAC
  AirPlayAacDecoder aac_decoder_;
  uint32_t aac_buffer_size_{0};
#endif
  bool airplay2_{false};
  bool task_stack_in_psram_{false};

  AirPlayMediaSource *media_source_{nullptr};
#ifdef USE_AIRPLAY_ARTWORK
  AirPlayImage *artwork_image_{nullptr};
#endif

  ::airplay::MbedtlsCrypto crypto_;
  ::airplay::SocketRunner runner_;
  std::unique_ptr<::airplay::Receiver> receiver_;
  StaticTask airplay_task_;
  StaticTask remote_task_;
  SemaphoreHandle_t lock_{nullptr};

  RemoteState remote_{};  // Guarded by lock_
  // Remote control task only
  char volume_dacp_id_[::airplay::RemoteTarget::DACP_ID_SIZE]{};
  AbsoluteVolume absolute_volume_{ABSOLUTE_VOLUME_UNKNOWN};

  // AirPlay task only: the rest of a frame the output took only in part
  uint8_t partial_frame_[::airplay::BYTES_PER_FRAME]{};
  size_t partial_remaining_{0};

  // Cross-thread state
  std::atomic<uint8_t> session_state_{0};
  std::atomic<bool> session_changed_{false};
  std::atomic<uint8_t> remote_requests_{0};
  std::atomic<float> remote_volume_{0.0f};
  // Volume last set by the sender, 0 to 1, to avoid sending it back; negative without a session
  std::atomic<float> sender_volume_{-1.0f};
  // Volume the sender last reported, including the steps of an adjustment from here; negative without a session
  std::atomic<float> reported_volume_{-1.0f};
  // The remote control task is moving the sender's volume; the sender's reports are not applied meanwhile
  std::atomic<bool> volume_adjusting_{false};

#ifdef USE_AIRPLAY_METADATA
  ::airplay::Metadata metadata_{};  // Guarded by lock_
  std::atomic<bool> metadata_changed_{false};
  LazyCallbackManager<void(const ::airplay::Metadata &)> metadata_callback_;
#endif

#ifdef USE_AIRPLAY_PROGRESS
  void update_progress_clock_();

  ::airplay::Progress progress_{};  // Guarded by lock_
  std::atomic<bool> progress_changed_{false};
  // Main loop only: the position was progress_base_ms_ at progress_since_ms_, and advances while running
  uint32_t progress_base_ms_{0};
  uint32_t progress_since_ms_{0};
  uint32_t progress_duration_ms_{0};
  bool progress_running_{false};
  LazyCallbackManager<void()> progress_callback_;
#endif

  bool wifi_streaming_{false};  // Main loop only
};

/// Base class of the hub's children
class AirPlayChild : public Component, public Parented<AirPlayHub> {
 public:
  float get_setup_priority() const override { return airplay_priority::CHILD; }
};

/// Base class of the hub's children that poll
class AirPlayPollingChild : public PollingComponent, public Parented<AirPlayHub> {
 public:
  float get_setup_priority() const override { return airplay_priority::CHILD; }
};

}  // namespace esphome::airplay_

#endif  // USE_ESP32

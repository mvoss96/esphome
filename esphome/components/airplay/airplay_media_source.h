#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"
#include "esphome/components/media_source/media_source.h"
#include "esphome/core/component.h"
#include "esphome/core/static_task.h"

#ifdef USE_AIRPLAY_METADATA
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <airplay.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace esphome::airplay {

/// AirPlay 1 (RAOP) receiver that feeds the received audio to a media player as a media source. The protocol is
/// handled by the airplay-cpp library; this class connects it to the media source, mDNS, Wi-Fi and text sensors.
class AirPlayMediaSource final : public Component,
                                 public media_source::MediaSource,
                                 public ::airplay::AudioSink,
                                 public ::airplay::Listener {
 public:
  AirPlayMediaSource(const char *name, uint16_t port, uint32_t buffer_duration_ms)
      : name_(name), buffer_duration_ms_(buffer_duration_ms), port_(port) {}

  void setup() override;
  void loop() override;
  void dump_config() override;
  // Registers its mDNS service, so it must run after the mdns component
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_task_stack_in_psram(bool task_stack_in_psram) { this->task_stack_in_psram_ = task_stack_in_psram; }

#ifdef USE_AIRPLAY_METADATA
  void set_title_text_sensor(text_sensor::TextSensor *sensor) { this->title_text_sensor_ = sensor; }
  void set_artist_text_sensor(text_sensor::TextSensor *sensor) { this->artist_text_sensor_ = sensor; }
  void set_album_text_sensor(text_sensor::TextSensor *sensor) { this->album_text_sensor_ = sensor; }
#endif

  // MediaSource interface
  bool play_uri(const std::string &uri) override;
  void handle_command(media_source::MediaSourceCommand command) override;
  bool can_handle(const std::string &uri) const override;
  // Next and previous are forwarded to the sender
  bool has_internal_playlist() const override { return true; }
  void notify_volume_changed(float volume) override;
  void notify_audio_played(uint32_t frames, int64_t timestamp) override;

  // airplay::AudioSink, called by the AirPlay task
  size_t write(const int16_t *samples, size_t frames) override;

  // airplay::Listener, called by the AirPlay task
  void on_session_state(::airplay::SessionState state) override;
  void on_volume(float volume) override;
  void on_mute(bool muted) override;
  void on_metadata(const ::airplay::Metadata &metadata) override;
  void on_remote_target(const ::airplay::RemoteTarget &target) override;

 protected:
  /// Requests to the remote control task, sent as bits of an atomic
  enum RemoteRequest : uint8_t {
    REMOTE_REQUEST_DISCOVER = 1 << 0,
    REMOTE_REQUEST_PLAY = 1 << 1,
    REMOTE_REQUEST_PAUSE = 1 << 2,
    REMOTE_REQUEST_NEXT = 1 << 3,
    REMOTE_REQUEST_PREVIOUS = 1 << 4,
    REMOTE_REQUEST_VOLUME = 1 << 5,
  };

  /// The sender's remote control (DACP) details, shared between the AirPlay and remote control tasks
  struct RemoteState {
    ::airplay::RemoteTarget target;
    uint16_t port;  // 0 until found through mDNS
    bool session_active;
    uint32_t session_end_ms;
  };

  // AirPlay task: drives the receiver (airplay_media_source.cpp)
  static void airplay_task(void *params);

  // Remote control task (airplay_remote.cpp)
  static void remote_task(void *params);
  void remote_task_loop_();
  void send_remote_request_(uint8_t request);
  bool discover_remote_port_(RemoteState &remote);

#ifdef USE_AIRPLAY_METADATA
  void publish_metadata_();

  text_sensor::TextSensor *title_text_sensor_{nullptr};
  text_sensor::TextSensor *artist_text_sensor_{nullptr};
  text_sensor::TextSensor *album_text_sensor_{nullptr};
  ::airplay::Metadata metadata_{};  // Guarded by lock_
  std::atomic<bool> metadata_changed_{false};
#endif

  void register_mdns_service_();
  void set_wifi_streaming_(bool streaming);

  // Configuration
  const char *name_;
  uint32_t buffer_duration_ms_;
  uint16_t port_;
  bool task_stack_in_psram_{false};

  ::airplay::MbedtlsCrypto crypto_;
  ::airplay::SocketRunner runner_;
  std::unique_ptr<::airplay::Receiver> receiver_;

  StaticTask airplay_task_;
  StaticTask remote_task_;
  SemaphoreHandle_t lock_{nullptr};

  // Cross-thread state
  std::atomic<uint8_t> session_state_{static_cast<uint8_t>(::airplay::SessionState::NONE)};
  std::atomic<uint8_t> remote_requests_{0};
  std::atomic<float> remote_volume_{0.0f};
  // Volume last set by the sender, 0 to 1, to avoid sending it back; negative without a session
  std::atomic<float> sender_volume_{-1.0f};

  // Main loop state
  bool play_requested_{false};
  bool wifi_streaming_{false};

  RemoteState remote_{};  // Guarded by lock_

  audio::AudioStreamInfo stream_info_{16, ::airplay::NUM_CHANNELS, ::airplay::SAMPLE_RATE};
};

}  // namespace esphome::airplay

#endif  // USE_ESP32

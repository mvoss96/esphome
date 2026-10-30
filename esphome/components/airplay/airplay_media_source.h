#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/components/media_source/media_source.h"
#include "esphome/core/component.h"
#include "esphome/core/static_task.h"

#ifdef USE_AIRPLAY_METADATA
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace esphome::airplay {

// Connects the airplay-cpp library to this component. Defined in the .cpp file only: the library's namespace is also
// called airplay, and every component header ends up in the generated main.cpp next to "using namespace esphome".
class AirPlayBridge;
#ifdef USE_AIRPLAY_ARTWORK
class AirPlayImage;
#endif

/// AirPlay 1 (RAOP) receiver that feeds the received audio to a media player as a media source. The protocol is
/// handled by the airplay-cpp library; this class connects it to the media source, mDNS, Wi-Fi and text sensors.
class AirPlayMediaSource final : public Component, public media_source::MediaSource {
 public:
  AirPlayMediaSource(const char *name, uint16_t port, uint32_t buffer_duration_ms);
  ~AirPlayMediaSource() override;

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
#ifdef USE_AIRPLAY_ARTWORK
  void set_artwork_image(AirPlayImage *image) { this->artwork_image_ = image; }
#endif

  // MediaSource interface
  bool play_uri(const std::string &uri) override;
  void handle_command(media_source::MediaSourceCommand command) override;
  bool can_handle(const std::string &uri) const override;
  // Next and previous are forwarded to the sender
  bool has_internal_playlist() const override { return true; }
  void notify_volume_changed(float volume) override;
  void notify_audio_played(uint32_t frames, int64_t timestamp) override;

 protected:
  friend class AirPlayBridge;
#ifdef USE_AIRPLAY_ARTWORK
  class AirPlayImage;
#endif

  /// Requests to the remote control task, sent as bits of an atomic
  enum RemoteRequest : uint8_t {
    REMOTE_REQUEST_DISCOVER = 1 << 0,
    REMOTE_REQUEST_PLAY = 1 << 1,
    REMOTE_REQUEST_PAUSE = 1 << 2,
    REMOTE_REQUEST_NEXT = 1 << 3,
    REMOTE_REQUEST_PREVIOUS = 1 << 4,
    REMOTE_REQUEST_VOLUME = 1 << 5,
  };

  /// Session state, as airplay::SessionState
  enum SessionState : uint8_t {
    SESSION_STATE_NONE = 0,
    SESSION_STATE_PLAYING,
    SESSION_STATE_PAUSED,
  };

  static void airplay_task(void *params);
  static void remote_task(void *params);
  void send_remote_request_(uint8_t request);
  void register_mdns_service_();
  void set_wifi_streaming_(bool streaming);
#ifdef USE_AIRPLAY_METADATA
  void publish_metadata_();

  static constexpr size_t METADATA_FIELD_SIZE = 128;
  struct TrackInfo {
    char title[METADATA_FIELD_SIZE];
    char artist[METADATA_FIELD_SIZE];
    char album[METADATA_FIELD_SIZE];
  };

  text_sensor::TextSensor *title_text_sensor_{nullptr};
  text_sensor::TextSensor *artist_text_sensor_{nullptr};
  text_sensor::TextSensor *album_text_sensor_{nullptr};
  TrackInfo track_info_{};  // Guarded by lock_
  std::atomic<bool> metadata_changed_{false};
#endif
#ifdef USE_AIRPLAY_ARTWORK
  AirPlayImage *artwork_image_{nullptr};
#endif

  // Configuration
  const char *name_;
  uint32_t buffer_duration_ms_;
  uint16_t port_;
  bool task_stack_in_psram_{false};

  std::unique_ptr<AirPlayBridge> bridge_;
  StaticTask airplay_task_;
  StaticTask remote_task_;
  SemaphoreHandle_t lock_{nullptr};

  // Cross-thread state
  std::atomic<uint8_t> session_state_{SESSION_STATE_NONE};
  std::atomic<uint8_t> remote_requests_{0};
  std::atomic<float> remote_volume_{0.0f};
  // Volume last set by the sender, 0 to 1, to avoid sending it back; negative without a session
  std::atomic<float> sender_volume_{-1.0f};
  // Volume the sender last reported, including the steps of an adjustment from here; negative without a session
  std::atomic<float> reported_volume_{-1.0f};
  // The remote control task is moving the sender's volume; the sender's reports are not applied meanwhile
  std::atomic<bool> volume_adjusting_{false};

  // Main loop state
  bool play_requested_{false};
  bool wifi_streaming_{false};
};

}  // namespace esphome::airplay

#endif  // USE_ESP32

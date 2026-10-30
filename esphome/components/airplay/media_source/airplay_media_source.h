#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/components/airplay/airplay_hub.h"
#include "esphome/components/audio/audio.h"
#include "esphome/components/media_source/media_source.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome::airplay_ {

/// Feeds the audio of the AirPlay hub to a media player and passes the player's commands to the sender.
class AirPlayMediaSource final : public AirPlayChild, public media_source::MediaSource {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // MediaSource interface
  bool play_uri(const std::string &uri) override;
  void handle_command(media_source::MediaSourceCommand command) override;
  bool can_handle(const std::string &uri) const override;
  // Next and previous are forwarded to the sender
  bool has_internal_playlist() const override { return true; }
  void notify_volume_changed(float volume) override;
  void notify_audio_played(uint32_t frames, int64_t timestamp) override;

  // ===== From the hub, on the AirPlay task =====
  /// Writes interleaved 16-bit stereo audio; nothing unless this source is the active one
  size_t write_audio(const uint8_t *data, size_t length) {
    return this->write_output(data, length, 0, this->stream_info_);
  }
  void on_session_changed() { this->enable_loop_soon_any_context(); }
  void on_sender_volume(float volume) { this->request_volume_(volume); }
  void on_sender_mute(bool muted) { this->request_mute_(muted); }

 protected:
  audio::AudioStreamInfo stream_info_{16, ::airplay::NUM_CHANNELS, ::airplay::SAMPLE_RATE};
  bool play_requested_{false};
};

}  // namespace esphome::airplay_

#endif  // USE_ESP32

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
  /// Whether the AirPlay sender may have the speaker
  enum class SpeakerClaim : uint8_t {
    /// It may: its session takes the speaker when it starts playing. Also when no sender is connected.
    SPEAKER_CLAIM_OWNER,
    /// Another source took the speaker and the sender is paused: it takes the speaker back when it plays again
    SPEAKER_CLAIM_WAITING,
    /// Another source took the speaker and the sender has not paused, or was disconnected for it. It takes the
    /// speaker back only after it paused and plays again; connecting again by itself does not count.
    SPEAKER_CLAIM_DECLINED,
  };

  void set_claim_(SpeakerClaim claim);
  /// Takes the speaker away from the sender and pauses it, or disconnects a sender that cannot be paused
  void decline_sender_();
  void check_pause_();

  audio::AudioStreamInfo stream_info_{16, ::airplay::NUM_CHANNELS, ::airplay::SAMPLE_RATE};
  bool play_requested_{false};
  SpeakerClaim claim_{SpeakerClaim::SPEAKER_CLAIM_OWNER};
  // The sender that lost the speaker; any other sender that starts playing takes it
  uint32_t declined_address_{0};
  // When a declined sender disconnected; 0 while it is connected
  uint32_t declined_end_ms_{0};
};

}  // namespace esphome::airplay_

#endif  // USE_ESP32

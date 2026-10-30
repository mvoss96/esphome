#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

namespace esphome::airplay_ {

ESPHOME_LOG_TAG(TAG, "airplay.media_source");

static constexpr char URI_PREFIX[] = "airplay://";
static constexpr char PLAY_URI[] = "airplay://current";

static constexpr uint32_t PLAY_RETRY_ID = 0;
static constexpr uint32_t PLAY_RETRY_MS = 3000;

void AirPlayMediaSource::setup() { this->disable_loop(); }

void AirPlayMediaSource::dump_config() { ESP_LOGCONFIG(TAG, "AirPlay Media Source"); }

// THREAD CONTEXT: Main loop. Woken by the hub when the session changes.
void AirPlayMediaSource::loop() {
  const ::airplay::SessionState desired = this->parent_->get_session_state();
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
  this->disable_loop();
}

bool AirPlayMediaSource::can_handle(const std::string &uri) const { return uri.starts_with(URI_PREFIX); }

// THREAD CONTEXT: Main loop
bool AirPlayMediaSource::play_uri(const std::string &uri) {
  this->play_requested_ = false;
  if (this->parent_->is_failed() || !this->can_handle(uri))
    return false;

  const ::airplay::SessionState desired = this->parent_->get_session_state();
  if (desired == ::airplay::SessionState::NONE) {
    // Accept and stay idle. Failing would make the orchestrator retry endlessly when repeat is enabled.
    ESP_LOGD(TAG, "No AirPlay sender is connected");
    return true;
  }
  // The orchestrator restarts its count of played frames right after this call
  this->parent_->request_output_restart();
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
      if (this->parent_->get_session_state() == ::airplay::SessionState::NONE) {
        this->play_requested_ = false;
        this->set_state_(media_source::MediaSourceState::IDLE);
      } else if (!this->parent_->is_failed()) {
        this->parent_->request_disconnect();
      }
      return;
    case media_source::MediaSourceCommand::PLAY:
      request = AirPlayHub::REMOTE_REQUEST_PLAY;
      break;
    case media_source::MediaSourceCommand::PAUSE:
      request = AirPlayHub::REMOTE_REQUEST_PAUSE;
      break;
    case media_source::MediaSourceCommand::NEXT:
      request = AirPlayHub::REMOTE_REQUEST_NEXT;
      break;
    case media_source::MediaSourceCommand::PREVIOUS:
      request = AirPlayHub::REMOTE_REQUEST_PREVIOUS;
      break;
    default:
      return;
  }
  // Also works for a while after the sender disconnected: "play" then makes it reconnect
  this->parent_->send_remote_request(request);
}

// THREAD CONTEXT: Main loop
void AirPlayMediaSource::notify_volume_changed(float volume) { this->parent_->set_speaker_volume(volume); }

// THREAD CONTEXT: Speaker playback callback
void AirPlayMediaSource::notify_audio_played(uint32_t frames, int64_t timestamp) {
  this->parent_->notify_played(frames, timestamp);
}

}  // namespace esphome::airplay_

#endif  // USE_ESP32

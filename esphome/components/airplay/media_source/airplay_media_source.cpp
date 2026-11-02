#include "airplay_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <algorithm>

namespace esphome::airplay_ {

ESPHOME_LOG_TAG(TAG, "airplay.media_source");

static constexpr char URI_PREFIX[] = "airplay://";
static constexpr char PLAY_URI[] = "airplay://current";

static constexpr uint32_t PLAY_RETRY_ID = 0;
static constexpr uint32_t PLAY_RETRY_MS = 3000;
static constexpr uint32_t PAUSE_CHECK_ID = 1;
/// A sender that has not paused this long after being asked to is disconnected instead. A realtime sender that
/// pauses just stops sending, which the receiver notices after 3 s.
static constexpr uint32_t PAUSE_CHECK_MS = 6000;
/// A sender that was disconnected and connects again within this time did so by itself (Music Assistant does)
static constexpr uint32_t AUTO_RECONNECT_MS = 30000;

void AirPlayMediaSource::setup() { this->disable_loop(); }

void AirPlayMediaSource::dump_config() { ESP_LOGCONFIG(TAG, "AirPlay Media Source"); }

// THREAD CONTEXT: Main loop. Woken by the hub when the session changes.
void AirPlayMediaSource::loop() {
  const ::airplay::SessionState desired = this->parent_->get_session_state();
  const media_source::MediaSourceState state = this->get_state();

  if (this->claim_ != SpeakerClaim::SPEAKER_CLAIM_OWNER) {
    // The last start wins: another sender that starts playing, or this one when it plays again after a pause
    if (desired == ::airplay::SessionState::NONE) {
      if (this->claim_ == SpeakerClaim::SPEAKER_CLAIM_WAITING) {
        // A paused sender that left: when it connects again, it was started again
        this->set_claim_(SpeakerClaim::SPEAKER_CLAIM_OWNER);
      } else if (this->declined_end_ms_ == 0) {
        this->declined_end_ms_ = std::max<uint32_t>(App.get_loop_component_start_time(), 1);
      }
    } else if (desired == ::airplay::SessionState::PAUSED) {
      this->set_claim_(SpeakerClaim::SPEAKER_CLAIM_WAITING);
    } else if (this->claim_ == SpeakerClaim::SPEAKER_CLAIM_WAITING ||
               this->parent_->get_sender_address() != this->declined_address_) {
      ESP_LOGD(TAG, "A sender plays again");
      this->set_claim_(SpeakerClaim::SPEAKER_CLAIM_OWNER);
    } else if (this->declined_end_ms_ != 0) {
      // The declined sender connected again and plays: started again, unless it did so right away by itself
      if (App.get_loop_component_start_time() - this->declined_end_ms_ >= AUTO_RECONNECT_MS) {
        ESP_LOGD(TAG, "A sender plays again");
        this->set_claim_(SpeakerClaim::SPEAKER_CLAIM_OWNER);
      } else {
        ESP_LOGD(TAG, "The sender connected again by itself; it stays without the speaker");
        this->declined_end_ms_ = 0;
      }
    }
    if (this->claim_ != SpeakerClaim::SPEAKER_CLAIM_OWNER) {
      this->disable_loop();
      return;
    }
  }

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
  // Chosen on the device: the sender gets the speaker back
  this->set_claim_(SpeakerClaim::SPEAKER_CLAIM_OWNER);
  // The orchestrator restarts its count of played frames right after this call
  this->parent_->request_output_restart();
  this->parent_->send_speaker_volume();
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
      } else if (this->parent_->is_failed()) {
        return;
      } else {
        this->decline_sender_();
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
void AirPlayMediaSource::decline_sender_() {
  if (!this->parent_->pause_sender()) {
    // AirPlay 1
    this->parent_->request_disconnect();
    return;
  }
  // AirPlay 2: iOS keeps its route to a receiver that disconnects and does not connect again by itself. Paused
  // instead, the sender stays connected and can resume here.
  ESP_LOGI(TAG, "Pausing the sender");
  this->declined_address_ = this->parent_->get_sender_address();
  this->declined_end_ms_ = 0;
  this->set_claim_(this->parent_->get_session_state() == ::airplay::SessionState::PAUSED
                       ? SpeakerClaim::SPEAKER_CLAIM_WAITING
                       : SpeakerClaim::SPEAKER_CLAIM_DECLINED);
  this->play_requested_ = false;
  this->cancel_timeout(PLAY_RETRY_ID);
  this->set_state_(media_source::MediaSourceState::IDLE);
  this->set_timeout(PAUSE_CHECK_ID, PAUSE_CHECK_MS, [this]() { this->check_pause_(); });
}

// THREAD CONTEXT: Main loop
void AirPlayMediaSource::check_pause_() {
  if (this->claim_ != SpeakerClaim::SPEAKER_CLAIM_DECLINED || this->declined_end_ms_ != 0 ||
      this->parent_->get_session_state() == ::airplay::SessionState::NONE)
    return;
  ESP_LOGI(TAG, "The sender did not pause, disconnecting it");
  this->parent_->request_disconnect();
}

// THREAD CONTEXT: Main loop
void AirPlayMediaSource::set_claim_(SpeakerClaim claim) {
  if (claim == this->claim_)
    return;
  this->claim_ = claim;
  const bool owner = claim == SpeakerClaim::SPEAKER_CLAIM_OWNER;
  // A sender without the speaker stays connected, but its audio is dropped as it arrives
  this->parent_->set_audio_held(!owner);
  if (owner) {
    this->declined_end_ms_ = 0;
    this->cancel_timeout(PAUSE_CHECK_ID);
  }
}

// THREAD CONTEXT: Main loop. The player tells every source; only the sender that has the speaker follows it.
void AirPlayMediaSource::notify_volume_changed(float volume) {
  this->parent_->set_speaker_volume(volume, this->get_state() != media_source::MediaSourceState::IDLE);
}

// THREAD CONTEXT: Speaker playback callback
void AirPlayMediaSource::notify_audio_played(uint32_t frames, int64_t timestamp) {
  this->parent_->notify_played(frames, timestamp);
}

}  // namespace esphome::airplay_

#endif  // USE_ESP32

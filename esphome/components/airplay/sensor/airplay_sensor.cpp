#include "airplay_sensor.h"

#if defined(USE_ESP32) && defined(USE_AIRPLAY_PROGRESS)

#include "esphome/core/log.h"

#include <cmath>

namespace esphome::airplay_ {

ESPHOME_LOG_TAG(TAG, "airplay.sensor");

void AirPlayTrackProgressSensor::setup() {
  this->parent_->add_progress_callback([this]() {
    if (this->parent_->get_track_duration_ms() == 0) {
      this->stop_poller();
      if (!std::isnan(this->get_raw_state()))
        this->publish_state(NAN);
      return;
    }
    this->publish_state(this->parent_->get_track_progress_ms());
    if (this->parent_->is_progress_running()) {
      this->start_poller();
    } else {
      this->stop_poller();
    }
  });
  this->stop_poller();
}

void AirPlayTrackProgressSensor::update() { this->publish_state(this->parent_->get_track_progress_ms()); }

void AirPlayTrackProgressSensor::dump_config() {
  LOG_SENSOR("", "AirPlay Track Progress", this);
  LOG_UPDATE_INTERVAL(this);
}

void AirPlayTrackDurationSensor::setup() {
  this->parent_->add_progress_callback([this]() {
    const uint32_t duration = this->parent_->get_track_duration_ms();
    const float value = duration > 0 ? static_cast<float>(duration) : NAN;
    const float current = this->get_raw_state();
    if (current != value && !(std::isnan(current) && std::isnan(value)))
      this->publish_state(value);
  });
}

void AirPlayTrackDurationSensor::dump_config() { LOG_SENSOR("", "AirPlay Track Duration", this); }

}  // namespace esphome::airplay_

#endif  // USE_ESP32 && USE_AIRPLAY_PROGRESS

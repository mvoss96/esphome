#include "airplay_sensor.h"

#if defined(USE_ESP32) && (defined(USE_AIRPLAY_PROGRESS) || defined(USE_AIRPLAY_METADATA))

#include "esphome/core/log.h"

#include <cmath>

namespace esphome::airplay_ {

ESPHOME_LOG_TAG(TAG, "airplay.sensor");

/// Publishes only changes; NAN counts as equal to NAN
static void publish_if_changed(sensor::Sensor *sensor, float value) {
  const float current = sensor->get_raw_state();
  if (current != value && !(std::isnan(current) && std::isnan(value)))
    sensor->publish_state(value);
}

#ifdef USE_AIRPLAY_PROGRESS
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
    publish_if_changed(this, duration > 0 ? static_cast<float>(duration) : NAN);
  });
}

void AirPlayTrackDurationSensor::dump_config() { LOG_SENSOR("", "AirPlay Track Duration", this); }
#endif  // USE_AIRPLAY_PROGRESS

#ifdef USE_AIRPLAY_METADATA
void AirPlayMetadataSensor::setup() {
  this->parent_->add_metadata_callback([this](const ::airplay::Metadata &metadata) {
    const uint16_t value = this->metadata_type_ == AirPlayNumericMetadataType::AIRPLAY_NUMERIC_METADATA_TYPE_TRACK
                               ? metadata.track
                               : metadata.year;
    publish_if_changed(this, value > 0 ? static_cast<float>(value) : NAN);
  });
}

void AirPlayMetadataSensor::dump_config() {
  LOG_SENSOR("",
             this->metadata_type_ == AirPlayNumericMetadataType::AIRPLAY_NUMERIC_METADATA_TYPE_TRACK ? "AirPlay Track"
                                                                                                     : "AirPlay Year",
             this);
}
#endif  // USE_AIRPLAY_METADATA

}  // namespace esphome::airplay_

#endif  // USE_ESP32 && (USE_AIRPLAY_PROGRESS || USE_AIRPLAY_METADATA)

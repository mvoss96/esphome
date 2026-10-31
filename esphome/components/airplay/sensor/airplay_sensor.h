#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && (defined(USE_AIRPLAY_PROGRESS) || defined(USE_AIRPLAY_METADATA))

#include "esphome/components/airplay/airplay_hub.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome::airplay_ {

#ifdef USE_AIRPLAY_PROGRESS
/// Position in the current track in milliseconds; unknown when no track plays. Updates at the update interval
/// while the track plays.
class AirPlayTrackProgressSensor final : public AirPlayPollingChild, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
};

/// Length of the current track in milliseconds; unknown when no track plays
class AirPlayTrackDurationSensor final : public AirPlayChild, public sensor::Sensor {
 public:
  void setup() override;
  void dump_config() override;
};
#endif  // USE_AIRPLAY_PROGRESS

#ifdef USE_AIRPLAY_METADATA
enum class AirPlayNumericMetadataType : uint8_t {
  AIRPLAY_NUMERIC_METADATA_TYPE_YEAR,
  AIRPLAY_NUMERIC_METADATA_TYPE_TRACK,
};

/// Year or track number of the current track; unknown when the sender does not send it
class AirPlayMetadataSensor final : public AirPlayChild, public sensor::Sensor {
 public:
  void setup() override;
  void dump_config() override;

  void set_metadata_type(AirPlayNumericMetadataType metadata_type) { this->metadata_type_ = metadata_type; }

 protected:
  AirPlayNumericMetadataType metadata_type_{AirPlayNumericMetadataType::AIRPLAY_NUMERIC_METADATA_TYPE_YEAR};
};
#endif  // USE_AIRPLAY_METADATA

}  // namespace esphome::airplay_

#endif  // USE_ESP32 && (USE_AIRPLAY_PROGRESS || USE_AIRPLAY_METADATA)

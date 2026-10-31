#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_AIRPLAY_PROGRESS)

#include "esphome/components/airplay/airplay_hub.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome::airplay_ {

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

}  // namespace esphome::airplay_

#endif  // USE_ESP32 && USE_AIRPLAY_PROGRESS

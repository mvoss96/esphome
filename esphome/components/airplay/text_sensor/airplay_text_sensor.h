#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_AIRPLAY_METADATA)

#include "esphome/components/airplay/airplay_hub.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome::airplay_ {

enum class AirPlayMetadataType : uint8_t {
  AIRPLAY_METADATA_TYPE_TITLE,
  AIRPLAY_METADATA_TYPE_ARTIST,
  AIRPLAY_METADATA_TYPE_ALBUM,
  AIRPLAY_METADATA_TYPE_ALBUM_ARTIST,
};

/// A detail of the track that plays over AirPlay; empty when no track plays
class AirPlayTextSensor final : public AirPlayChild, public text_sensor::TextSensor {
 public:
  void setup() override;
  void dump_config() override;

  void set_metadata_type(AirPlayMetadataType metadata_type) { this->metadata_type_ = metadata_type; }

 protected:
  AirPlayMetadataType metadata_type_{AirPlayMetadataType::AIRPLAY_METADATA_TYPE_TITLE};
};

}  // namespace esphome::airplay_

#endif  // USE_ESP32 && USE_AIRPLAY_METADATA

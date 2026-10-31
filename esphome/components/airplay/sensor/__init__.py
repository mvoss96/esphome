import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_TYPE,
    CONF_YEAR,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLISECOND,
)
from esphome.types import ConfigType

from .. import CONF_AIRPLAY_ID, AirPlayHub, airplay_ns

CODEOWNERS = ["@mvoss96"]
DEPENDENCIES = ["airplay"]

CONF_TRACK = "track"
CONF_TRACK_DURATION = "track_duration"
CONF_TRACK_PROGRESS = "track_progress"

AirPlayTrackProgressSensor = airplay_ns.class_(
    "AirPlayTrackProgressSensor", sensor.Sensor, cg.PollingComponent
)
AirPlayTrackDurationSensor = airplay_ns.class_(
    "AirPlayTrackDurationSensor", sensor.Sensor, cg.Component
)
AirPlayMetadataSensor = airplay_ns.class_(
    "AirPlayMetadataSensor", sensor.Sensor, cg.Component
)

AirPlayNumericMetadataType = airplay_ns.enum(
    "AirPlayNumericMetadataType", is_class=True
)
NUMERIC_METADATA_TYPES = {
    CONF_YEAR: AirPlayNumericMetadataType.AIRPLAY_NUMERIC_METADATA_TYPE_YEAR,
    CONF_TRACK: AirPlayNumericMetadataType.AIRPLAY_NUMERIC_METADATA_TYPE_TRACK,
}
PROGRESS_TYPES = (CONF_TRACK_PROGRESS, CONF_TRACK_DURATION)

_HUB_ID_SCHEMA = cv.Schema({cv.GenerateID(CONF_AIRPLAY_ID): cv.use_id(AirPlayHub)})

CONFIG_SCHEMA = cv.All(
    cv.typed_schema(
        {
            CONF_TRACK_PROGRESS: sensor.sensor_schema(
                AirPlayTrackProgressSensor,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                unit_of_measurement=UNIT_MILLISECOND,
                icon="mdi:progress-clock",
            )
            .extend(_HUB_ID_SCHEMA)
            .extend(cv.polling_component_schema("1s")),
            CONF_TRACK_DURATION: sensor.sensor_schema(
                AirPlayTrackDurationSensor,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                unit_of_measurement=UNIT_MILLISECOND,
                icon="mdi:timer-music-outline",
            )
            .extend(_HUB_ID_SCHEMA)
            .extend(cv.COMPONENT_SCHEMA),
            **{
                key: sensor.sensor_schema(
                    AirPlayMetadataSensor,
                    accuracy_decimals=0,
                    icon=icon,
                )
                .extend(_HUB_ID_SCHEMA)
                .extend(cv.COMPONENT_SCHEMA)
                for key, icon in (
                    (CONF_YEAR, "mdi:calendar"),
                    (CONF_TRACK, "mdi:pound"),
                )
            },
        },
        key=CONF_TYPE,
    ),
    cv.only_on_esp32,
)


async def to_code(config: ConfigType) -> None:
    # Also makes the receiver ask senders for the track position or details
    if config[CONF_TYPE] in PROGRESS_TYPES:
        cg.add_define("USE_AIRPLAY_PROGRESS")
    else:
        cg.add_define("USE_AIRPLAY_METADATA")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_AIRPLAY_ID])
    await sensor.register_sensor(var, config)
    if (metadata_type := NUMERIC_METADATA_TYPES.get(config[CONF_TYPE])) is not None:
        cg.add(var.set_metadata_type(metadata_type))

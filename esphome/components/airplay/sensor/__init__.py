import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TYPE, STATE_CLASS_MEASUREMENT, UNIT_MILLISECOND
from esphome.types import ConfigType

from .. import CONF_AIRPLAY_ID, AirPlayHub, airplay_ns

CODEOWNERS = ["@mvoss96"]
DEPENDENCIES = ["airplay"]

CONF_TRACK_DURATION = "track_duration"
CONF_TRACK_PROGRESS = "track_progress"

AirPlayTrackProgressSensor = airplay_ns.class_(
    "AirPlayTrackProgressSensor", sensor.Sensor, cg.PollingComponent
)
AirPlayTrackDurationSensor = airplay_ns.class_(
    "AirPlayTrackDurationSensor", sensor.Sensor, cg.Component
)

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
        },
        key=CONF_TYPE,
    ),
    cv.only_on_esp32,
)


async def to_code(config: ConfigType) -> None:
    # Also makes the receiver ask senders for the track position
    cg.add_define("USE_AIRPLAY_PROGRESS")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_AIRPLAY_ID])
    await sensor.register_sensor(var, config)

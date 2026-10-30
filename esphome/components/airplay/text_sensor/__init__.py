import esphome.codegen as cg
from esphome.components import text_sensor
from esphome.components.const import CONF_TITLE
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TYPE
from esphome.types import ConfigType

from .. import CONF_AIRPLAY_ID, AirPlayHub, airplay_ns

CODEOWNERS = ["@mvoss96"]
DEPENDENCIES = ["airplay"]

CONF_ALBUM = "album"
CONF_ARTIST = "artist"

AirPlayTextSensor = airplay_ns.class_(
    "AirPlayTextSensor", text_sensor.TextSensor, cg.Component
)

AirPlayMetadataType = airplay_ns.enum("AirPlayMetadataType", is_class=True)
METADATA_TYPES = {
    CONF_TITLE: AirPlayMetadataType.AIRPLAY_METADATA_TYPE_TITLE,
    CONF_ARTIST: AirPlayMetadataType.AIRPLAY_METADATA_TYPE_ARTIST,
    CONF_ALBUM: AirPlayMetadataType.AIRPLAY_METADATA_TYPE_ALBUM,
}
ICONS = {
    CONF_TITLE: "mdi:music",
    CONF_ARTIST: "mdi:account-music",
    CONF_ALBUM: "mdi:album",
}

_HUB_ID_SCHEMA = cv.Schema({cv.GenerateID(CONF_AIRPLAY_ID): cv.use_id(AirPlayHub)})

CONFIG_SCHEMA = cv.All(
    cv.typed_schema(
        {
            key: text_sensor.text_sensor_schema(AirPlayTextSensor, icon=icon).extend(
                _HUB_ID_SCHEMA
            )
            for key, icon in ICONS.items()
        },
        key=CONF_TYPE,
    ),
    cv.only_on_esp32,
)


async def to_code(config: ConfigType) -> None:
    # Also makes the receiver ask senders for track details
    cg.add_define("USE_AIRPLAY_METADATA")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_AIRPLAY_ID])
    await text_sensor.register_text_sensor(var, config)
    cg.add(var.set_metadata_type(METADATA_TYPES[config[CONF_TYPE]]))

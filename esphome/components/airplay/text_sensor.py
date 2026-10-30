import esphome.codegen as cg
from esphome.components import text_sensor
from esphome.components.const import CONF_TITLE
import esphome.config_validation as cv
from esphome.types import ConfigType

from .media_source import AirPlayMediaSource

CODEOWNERS = ["@mvoss96"]
DEPENDENCIES = ["media_source"]

CONF_AIRPLAY_ID = "airplay_id"
CONF_ALBUM = "album"
CONF_ARTIST = "artist"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AIRPLAY_ID): cv.use_id(AirPlayMediaSource),
        cv.Optional(CONF_TITLE): text_sensor.text_sensor_schema(icon="mdi:music"),
        cv.Optional(CONF_ARTIST): text_sensor.text_sensor_schema(
            icon="mdi:account-music"
        ),
        cv.Optional(CONF_ALBUM): text_sensor.text_sensor_schema(icon="mdi:album"),
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_AIRPLAY_ID])
    # Also makes the receiver ask senders for track information
    cg.add_define("USE_AIRPLAY_METADATA")
    text_sensors = text_sensor.sub_text_sensors(config)
    await text_sensors(CONF_TITLE, hub.set_title_text_sensor)
    await text_sensors(CONF_ARTIST, hub.set_artist_text_sensor)
    await text_sensors(CONF_ALBUM, hub.set_album_text_sensor)

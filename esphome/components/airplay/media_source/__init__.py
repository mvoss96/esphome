import esphome.codegen as cg
from esphome.components import media_source, psram
import esphome.config_validation as cv
from esphome.const import CONF_BUFFER_DURATION, CONF_ID
from esphome.core import CORE, TimePeriod
from esphome.types import ConfigType

from .. import CONF_AIRPLAY_ID, AirPlayHub, airplay_ns

AUTO_LOAD = ["audio"]
CODEOWNERS = ["@mvoss96"]
DEPENDENCIES = ["airplay"]

# Synchronized playback buffers the sender's whole latency (about 2.25 s), which needs PSRAM.
# Without PSRAM the buffer is small and playback is not synchronized to the sender.
DEFAULT_BUFFER_DURATION_PSRAM_MS = 2500
DEFAULT_BUFFER_DURATION_MS = 500

AirPlayMediaSource = airplay_ns.class_(
    "AirPlayMediaSource", cg.Component, media_source.MediaSource
)

CONFIG_SCHEMA = cv.All(
    media_source.media_source_schema(AirPlayMediaSource).extend(
        {
            cv.GenerateID(CONF_AIRPLAY_ID): cv.use_id(AirPlayHub),
            cv.Optional(CONF_BUFFER_DURATION): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=TimePeriod(milliseconds=100), max=TimePeriod(milliseconds=5000)
                ),
            ),
        }
    ),
    cv.only_on_esp32,
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await media_source.register_media_source(var, config)
    hub = await cg.get_variable(config[CONF_AIRPLAY_ID])
    await cg.register_parented(var, hub)
    cg.add(hub.set_media_source(var))

    if (buffer_duration := config.get(CONF_BUFFER_DURATION)) is not None:
        buffer_duration_ms = int(buffer_duration.total_milliseconds)
    elif psram.DOMAIN in CORE.loaded_integrations:
        buffer_duration_ms = DEFAULT_BUFFER_DURATION_PSRAM_MS
    else:
        buffer_duration_ms = DEFAULT_BUFFER_DURATION_MS
    cg.add(hub.set_buffer_duration(buffer_duration_ms))

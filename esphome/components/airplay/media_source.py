import esphome.codegen as cg
from esphome.components import esp32, media_source, psram, socket, wifi
from esphome.components.mdns import DOMAIN as MDNS_DOMAIN
import esphome.config_validation as cv
from esphome.const import (
    CONF_BUFFER_DURATION,
    CONF_DISABLED,
    CONF_ID,
    CONF_NAME,
    CONF_PORT,
    CONF_TASK_STACK_IN_PSRAM,
)
from esphome.core import CORE, TimePeriod
import esphome.final_validate as fv
from esphome.types import ConfigType

from . import DOMAIN

AUTO_LOAD = ["audio", "mdns", "socket"]
CODEOWNERS = ["@mvoss96"]
DEPENDENCIES = ["network"]

airplay_ns = cg.esphome_ns.namespace("airplay")
AirPlayMediaSource = airplay_ns.class_(
    "AirPlayMediaSource", cg.Component, media_source.MediaSource
)

# The AirPlay protocol is handled by the airplay-cpp library
AIRPLAY_CPP_REPO = "https://github.com/mvoss96/airplay-cpp.git"
AIRPLAY_CPP_VERSION = "f7515838f033d5642c82f888a3e73b8efacb093b"

# Room for a burst of audio packets between two reads of a UDP socket
UDP_RECEIVE_QUEUE_SIZE = 32

# Synchronized playback buffers the sender's whole latency (about 2.25 s), which needs PSRAM.
# Without PSRAM the buffer is small and playback is not synchronized to the sender.
DEFAULT_BUFFER_DURATION_PSRAM_MS = 2500
DEFAULT_BUFFER_DURATION_MS = 500


def _set_default_name(config: ConfigType) -> ConfigType:
    if CONF_NAME not in config:
        config[CONF_NAME] = CORE.friendly_name or CORE.name
    return config


def _request_network_features(config: ConfigType) -> ConfigType:
    # Wi-Fi power saving and roaming scans cause dropouts while streaming
    wifi.enable_runtime_power_save_control()
    wifi.enable_runtime_roaming_suppression()
    return config


def _consume_sockets(config: ConfigType) -> ConfigType:
    # The RTSP server, two RTSP connections plus one remote control connection,
    # and the audio, control and timing sockets of a playback session
    socket.consume_sockets(1, DOMAIN, socket.SocketType.TCP_LISTEN)(config)
    socket.consume_sockets(3, DOMAIN)(config)
    socket.consume_sockets(3, DOMAIN, socket.SocketType.UDP)(config)
    return config


CONFIG_SCHEMA = cv.All(
    media_source.media_source_schema(AirPlayMediaSource)
    .extend(
        {
            cv.Optional(CONF_NAME): cv.All(cv.string_strict, cv.Length(min=1, max=48)),
            cv.Optional(CONF_PORT, default=5000): cv.port,
            cv.Optional(CONF_BUFFER_DURATION): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=TimePeriod(milliseconds=100), max=TimePeriod(milliseconds=5000)
                ),
            ),
            cv.Optional(CONF_TASK_STACK_IN_PSRAM): psram.validate_task_stack_in_psram,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _set_default_name,
    _consume_sockets,
    _request_network_features,
)


def _final_validate(config: ConfigType) -> ConfigType:
    mdns_config = fv.full_config.get().get(MDNS_DOMAIN, {})
    if mdns_config.get(CONF_DISABLED, False):
        raise cv.Invalid(
            "AirPlay senders find the receiver through mDNS, so the mdns component must not be disabled"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    if (buffer_duration := config.get(CONF_BUFFER_DURATION)) is not None:
        buffer_duration_ms = int(buffer_duration.total_milliseconds)
    elif psram.DOMAIN in CORE.loaded_integrations:
        buffer_duration_ms = DEFAULT_BUFFER_DURATION_PSRAM_MS
    else:
        buffer_duration_ms = DEFAULT_BUFFER_DURATION_MS
    var = cg.new_Pvariable(
        config[CONF_ID], config[CONF_NAME], config[CONF_PORT], buffer_duration_ms
    )
    await cg.register_component(var, config)
    await media_source.register_media_source(var, config)

    if config.get(CONF_TASK_STACK_IN_PSRAM):
        cg.add(var.set_task_stack_in_psram(True))
        psram.request_external_task_stack()

    esp32.add_idf_component(
        name="mvoss96/airplay-cpp", repo=AIRPLAY_CPP_REPO, ref=AIRPLAY_CPP_VERSION
    )
    esp32.add_idf_sdkconfig_option(
        "CONFIG_LWIP_UDP_RECVMBOX_SIZE", UDP_RECEIVE_QUEUE_SIZE
    )

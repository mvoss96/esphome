"""AirPlay receiver, built on the airplay-cpp library."""

import esphome.codegen as cg
from esphome.components import esp32, psram, socket, wifi
from esphome.components.mdns import DOMAIN as MDNS_DOMAIN
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISABLED,
    CONF_ESPHOME,
    CONF_ID,
    CONF_MODEL,
    CONF_NAME,
    CONF_PASSWORD,
    CONF_PLATFORM,
    CONF_PORT,
    CONF_PROJECT,
    CONF_TASK_STACK_IN_PSRAM,
)
from esphome.core import CORE
import esphome.final_validate as fv
from esphome.types import ConfigType

AUTO_LOAD = ["mdns", "socket"]
CODEOWNERS = ["@mvoss96"]
DEPENDENCIES = ["network"]
DOMAIN = "airplay"

CONF_AIRPLAY_ID = "airplay_id"
CONF_AIRPLAY2 = "airplay2"
CONF_AIRPLAY2_FEATURES = "airplay2_features"
CONF_AAC_BUFFER_SIZE = "aac_buffer_size"

# The AirPlay protocol is handled by the airplay-cpp library
AIRPLAY_CPP_REPO = "https://github.com/mvoss96/airplay-cpp.git"
AIRPLAY_CPP_VERSION = "00943fdd5214038bbab3aa8d81bd721639ae7830"

# Room for a burst of audio packets between two reads of a UDP socket
UDP_RECEIVE_QUEUE_SIZE = 32

# Trailing underscore avoids clashing with airplay-cpp's global `airplay` namespace
airplay_ns = cg.esphome_ns.namespace("airplay_")
AirPlayHub = airplay_ns.class_("AirPlayHub", cg.Component)


def _set_default_name(config: ConfigType) -> ConfigType:
    if CONF_NAME not in config:
        config[CONF_NAME] = CORE.friendly_name or CORE.name
    return config


def _validate_airplay2(config: ConfigType) -> ConfigType:
    if config[CONF_AIRPLAY2] and CONF_PASSWORD in config:
        raise cv.Invalid(
            f"'{CONF_AIRPLAY2}' cannot be used with '{CONF_PASSWORD}'",
            path=[CONF_AIRPLAY2],
        )
    return config


def _request_network_features(config: ConfigType) -> ConfigType:
    # Wi-Fi power saving and roaming scans cause dropouts while streaming
    wifi.enable_runtime_power_save_control()
    wifi.enable_runtime_roaming_suppression()
    return config


def _consume_sockets(config: ConfigType) -> ConfigType:
    # The RTSP server, two RTSP connections plus one remote control connection (and
    # the event and buffered audio connections of an AirPlay 2 sender), and the audio,
    # control and timing sockets of a playback session
    socket.consume_sockets(1, DOMAIN, socket.SocketType.TCP_LISTEN)(config)
    socket.consume_sockets(5 if config[CONF_AIRPLAY2] else 3, DOMAIN)(config)
    socket.consume_sockets(3, DOMAIN, socket.SocketType.UDP)(config)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AirPlayHub),
            cv.Optional(CONF_NAME): cv.All(cv.string_strict, cv.Length(min=1, max=48)),
            cv.Optional(CONF_MODEL): cv.All(cv.string_strict, cv.Length(min=1, max=64)),
            # Senders ask for it before they can stream
            cv.Optional(CONF_PASSWORD): cv.All(
                cv.string_strict, cv.Length(min=1, max=64)
            ),
            cv.Optional(CONF_PORT, default=5000): cv.port,
            # Experimental: AirPlay 2 with realtime audio; senders without AirPlay 1
            # (e.g. current iOS) need it
            cv.Optional(CONF_AIRPLAY2, default=False): cv.boolean,
            # For experiments: the AirPlay 2 feature bits to announce
            cv.Optional(CONF_AIRPLAY2_FEATURES): cv.hex_uint64_t,
            # Compressed audio that AirPlay 2 senders send ahead (AAC at 256 kbit/s
            # takes 32 KB per second); decoded into the media source's buffer
            cv.Optional(CONF_AAC_BUFFER_SIZE, default=131072): cv.int_range(
                min=32768, max=1048576
            ),
            cv.Optional(CONF_TASK_STACK_IN_PSRAM): psram.validate_task_stack_in_psram,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _validate_airplay2,
    _set_default_name,
    _consume_sockets,
    _request_network_features,
)


def _final_validate(config: ConfigType) -> ConfigType:
    full_config = fv.full_config.get()
    if full_config.get(MDNS_DOMAIN, {}).get(CONF_DISABLED, False):
        raise cv.Invalid(
            "AirPlay senders find the receiver through mDNS, so the mdns component must not be disabled"
        )
    sources = [
        source
        for source in full_config.get("media_source", [])
        if source.get(CONF_PLATFORM) == DOMAIN
        and source.get(CONF_AIRPLAY_ID) == config[CONF_ID]
    ]
    if len(sources) != 1:
        raise cv.Invalid(
            "The AirPlay receiver needs exactly one 'media_source' with 'platform: airplay'"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_NAME], config[CONF_PORT])
    await cg.register_component(var, config)

    # The model falls back to the project information, which is written as `manufacturer.model`
    project = CORE.config[CONF_ESPHOME].get(CONF_PROJECT, {})
    _, _, project_model = project.get(CONF_NAME, "").partition(".")
    if model := config.get(CONF_MODEL) or project_model:
        cg.add(var.set_model(model))

    if (password := config.get(CONF_PASSWORD)) is not None:
        cg.add(var.set_password(password))

    if config[CONF_AIRPLAY2]:
        cg.add(var.set_airplay2(True))
        # Buffered audio (AAC), which iOS uses for most apps
        cg.add_define("USE_AIRPLAY_AAC")
        cg.add(var.set_aac_buffer_size(config[CONF_AAC_BUFFER_SIZE]))
        esp32.add_idf_component(name="espressif/esp_audio_codec", ref="2.5.0")
    if (features := config.get(CONF_AIRPLAY2_FEATURES)) is not None:
        cg.add(var.set_airplay2_features(features))

    if config.get(CONF_TASK_STACK_IN_PSRAM):
        cg.add(var.set_task_stack_in_psram(True))
        psram.request_external_task_stack()

    esp32.add_idf_component(
        name="mvoss96/airplay-cpp", repo=AIRPLAY_CPP_REPO, ref=AIRPLAY_CPP_VERSION
    )
    esp32.add_idf_sdkconfig_option(
        "CONFIG_LWIP_UDP_RECVMBOX_SIZE", UDP_RECEIVE_QUEUE_SIZE
    )

"""Cover art of the track that plays over AirPlay, as an image (e.g. for LVGL)."""

from esphome import automation
import esphome.codegen as cg
from esphome.components import runtime_image
from esphome.components.image import CONF_TRANSPARENCY, Image_, add_metadata
import esphome.config_validation as cv
from esphome.const import CONF_FORMAT, CONF_ID, CONF_PLATFORM, CONF_RESIZE, CONF_TYPE
import esphome.final_validate as fv
from esphome.types import ConfigType

from .. import CONF_AIRPLAY_ID, AirPlayHub, airplay_ns

AUTO_LOAD = ["runtime_image"]
CODEOWNERS = ["@mvoss96"]
DEPENDENCIES = ["airplay"]

CONF_MAX_SIZE = "max_size"
CONF_ON_IMAGE_UPDATE = "on_image_update"
CONF_ON_IMAGE_CLEAR = "on_image_clear"
CONF_ON_IMAGE_ERROR = "on_image_error"

# Senders send JPEG or PNG; AUTO decodes both
FORMATS = ("AUTO", "JPEG", "JPG", "PNG")

# Cover art from Apple senders is usually 20 to 200 KB of JPEG
DEFAULT_MAX_SIZE = 256 * 1024
MAX_IMAGE_DIMENSION = 32767

AirPlayImage = airplay_ns.class_("AirPlayImage", cg.Component, Image_)

_CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(CONF_ON_IMAGE_UPDATE, "add_on_image_update_callback"),
    automation.CallbackAutomation(CONF_ON_IMAGE_CLEAR, "add_on_image_clear_callback"),
    automation.CallbackAutomation(CONF_ON_IMAGE_ERROR, "add_on_image_error_callback"),
)


def _validate_resize(config: ConfigType) -> ConfigType:
    width, height = config[CONF_RESIZE]
    if width > MAX_IMAGE_DIMENSION or height > MAX_IMAGE_DIMENSION:
        raise cv.Invalid(
            f"'{CONF_RESIZE}' width and height must be {MAX_IMAGE_DIMENSION} or less",
            path=[CONF_RESIZE],
        )
    return config


CONFIG_SCHEMA = cv.All(
    runtime_image.runtime_image_schema(AirPlayImage)
    .extend(
        {
            cv.GenerateID(CONF_AIRPLAY_ID): cv.use_id(AirPlayHub),
            cv.Optional(CONF_FORMAT, default="AUTO"): cv.one_of(*FORMATS, upper=True),
            # Images are scaled down to fit, keeping their aspect ratio
            cv.Required(CONF_RESIZE): cv.dimensions,
            cv.Optional(CONF_MAX_SIZE, default=DEFAULT_MAX_SIZE): cv.All(
                cv.validate_bytes, cv.int_range(min=1024, max=1024 * 1024)
            ),
            cv.Optional(CONF_ON_IMAGE_UPDATE): automation.validate_automation({}),
            cv.Optional(CONF_ON_IMAGE_CLEAR): automation.validate_automation({}),
            cv.Optional(CONF_ON_IMAGE_ERROR): automation.validate_automation({}),
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    runtime_image.validate_runtime_image_settings,
    _validate_resize,
    cv.only_on_esp32,
)


def _final_validate(config: ConfigType) -> ConfigType:
    images = fv.full_config.get().get("image", [])
    sources = [
        image[CONF_AIRPLAY_ID]
        for image in images
        if isinstance(image, dict) and image.get(CONF_PLATFORM) == "airplay"
    ]
    if sources.count(config[CONF_AIRPLAY_ID]) > 1:
        raise cv.Invalid("Each AirPlay receiver can have only one cover art image")
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    image_format = config[CONF_FORMAT]
    if image_format == "AUTO":
        # Only the formats senders use, rather than every format runtime_image knows
        runtime_image.enable_format("PNG")
        settings = await runtime_image.process_runtime_image_config(
            {**config, CONF_FORMAT: "JPEG"}
        )
        settings.format_enum = runtime_image.IMAGE_FORMAT_AUTO
    else:
        settings = await runtime_image.process_runtime_image_config(config)

    add_metadata(
        config[CONF_ID],
        settings.width,
        settings.height,
        config[CONF_TYPE],
        config[CONF_TRANSPARENCY],
    )
    var = cg.new_Pvariable(
        config[CONF_ID],
        settings.width,
        settings.height,
        settings.format_enum,
        settings.image_type_enum,
        settings.transparent,
        settings.placeholder or cg.nullptr,
        settings.byte_order_big_endian,
        config[CONF_MAX_SIZE],
    )
    await cg.register_component(var, config)
    await automation.build_callback_automations(var, config, _CALLBACK_AUTOMATIONS)

    cg.add_define("USE_AIRPLAY_ARTWORK")
    hub = await cg.get_variable(config[CONF_AIRPLAY_ID])
    await cg.register_parented(var, hub)
    cg.add(hub.set_artwork_image(var))

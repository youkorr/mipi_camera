import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_NAME
from esphome import pins

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c", "esp32"]
MULTI_CONF = True

mipi_camera_ns = cg.esphome_ns.namespace("mipi_camera")
MIPICameraComponent = mipi_camera_ns.class_("MIPICameraComponent", cg.Component, i2c.I2CDevice)

CONF_SENSOR_TYPE = "sensor_type"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_EXTERNAL_CLOCK_FREQUENCY = "external_clock_frequency"
CONF_RESET_PIN = "reset_pin"
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_FRAMERATE = "framerate"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_MIRROR = "mirror"
CONF_FLIP = "flip"

# Capteurs supportés
SUPPORTED_SENSORS = [
    "sc202cs",
    # Ajouter d'autres capteurs ici facilement
]

# Résolutions supportées
SUPPORTED_RESOLUTIONS = [
    "720P",   # 1280x720
    "VGA",    # 640x480
    "SVGA",   # 800x600
]

# Formats de pixels supportés
SUPPORTED_PIXEL_FORMATS = [
    "RGB565",
    "YUV422",
    "RAW8",
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MIPICameraComponent),
        cv.Optional(CONF_NAME): cv.string,
        cv.Required(CONF_SENSOR_TYPE): cv.one_of(*SUPPORTED_SENSORS, lower=True),
        cv.Optional(CONF_EXTERNAL_CLOCK_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_EXTERNAL_CLOCK_FREQUENCY, default=24000000): cv.int_range(
            min=6000000, max=40000000
        ),
        cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_RESOLUTION, default="720P"): cv.one_of(*SUPPORTED_RESOLUTIONS, upper=True),
        cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.one_of(*SUPPORTED_PIXEL_FORMATS, upper=True),
        cv.Optional(CONF_FRAMERATE, default=30): cv.int_range(min=1, max=60),
        cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
        cv.Optional(CONF_MIRROR, default=True): cv.boolean,
        cv.Optional(CONF_FLIP, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x36))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_sensor_type(config[CONF_SENSOR_TYPE]))

    if CONF_NAME in config:
        cg.add(var.set_name(config[CONF_NAME]))

    if CONF_EXTERNAL_CLOCK_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_EXTERNAL_CLOCK_PIN])
        cg.add(var.set_external_clock_pin(pin.get_pin()))
        cg.add(var.set_external_clock_frequency(config[CONF_EXTERNAL_CLOCK_FREQUENCY]))

    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))

    # Configuration résolution/format/framerate
    cg.add(var.set_resolution(config[CONF_RESOLUTION]))
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    
    # Configuration flip/mirror
    cg.add(var.set_mirror_enabled(config[CONF_MIRROR]))
    cg.add(var.set_flip_enabled(config[CONF_FLIP]))

    # Build flags pour ESP32-P4
    cg.add_build_flag("-DBOARD_HAS_PSRAM")
    cg.add_build_flag("-DCONFIG_CAMERA_CORE0=1")
    cg.add_build_flag("-DUSE_ESP32_VARIANT_ESP32P4")

    # Logs de compilation
    cg.add(
        cg.RawExpression(
            f'''
        ESP_LOGI("compile", "========================================");
        ESP_LOGI("compile", "MIPI Camera System v2.0");
        ESP_LOGI("compile", "  Name: {config.get(CONF_NAME, "N/A")}");
        ESP_LOGI("compile", "  Sensor: {config[CONF_SENSOR_TYPE]}");
        ESP_LOGI("compile", "  Resolution: {config[CONF_RESOLUTION]}");
        ESP_LOGI("compile", "  Format: {config[CONF_PIXEL_FORMAT]}");
        ESP_LOGI("compile", "  FPS: {config[CONF_FRAMERATE]}");
        ESP_LOGI("compile", "  Mirror: {config[CONF_MIRROR]}");
        ESP_LOGI("compile", "  Flip: {config[CONF_FLIP]}");
        ESP_LOGI("compile", "  Triple buffering enabled");
        ESP_LOGI("compile", "  CCM + AWB hardware support");
        ESP_LOGI("compile", "========================================");
    '''
        )
    )

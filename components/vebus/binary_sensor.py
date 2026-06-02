import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_CONNECTIVITY, DEVICE_CLASS_POWER

from . import CONF_VEBUS_ID, VEBusHub

CONF_SYNC = "sync"
CONF_DC_ALLOWS_INVERTING = "dc_allows_inverting"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VEBUS_ID): cv.use_id(VEBusHub),
        cv.Optional(CONF_SYNC): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
        ),
        cv.Optional(CONF_DC_ALLOWS_INVERTING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_POWER,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VEBUS_ID])
    if CONF_SYNC in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_SYNC])
        cg.add(hub.set_sync_binary_sensor(bs))
    if CONF_DC_ALLOWS_INVERTING in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_DC_ALLOWS_INVERTING])
        cg.add(hub.set_dc_ok_binary_sensor(bs))

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID

from . import CONF_VEBUS_ID, VEBusHub, vebus_ns

VEBusESSPowerNumber = vebus_ns.class_(
    "VEBusESSPowerNumber", number.Number, cg.Component
)

CONF_ESS_POWER = "ess_power"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VEBUS_ID): cv.use_id(VEBusHub),
        cv.Optional(CONF_ESS_POWER): number.number_schema(VEBusESSPowerNumber).extend(
            cv.COMPONENT_SCHEMA
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VEBUS_ID])
    if CONF_ESS_POWER in config:
        conf = config[CONF_ESS_POWER]
        var = cg.new_Pvariable(conf[CONF_ID])
        await number.register_number(
            var, conf, min_value=-1875, max_value=1875, step=1
        )
        await cg.register_component(var, conf)
        cg.add(var.set_parent(hub))

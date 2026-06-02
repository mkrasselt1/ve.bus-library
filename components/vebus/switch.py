import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import CONF_VEBUS_ID, VEBusHub, vebus_ns

VEBusVirtualModeSwitch = vebus_ns.class_(
    "VEBusVirtualModeSwitch", switch.Switch, cg.Component
)

CONF_VIRTUAL_MODE = "virtual_setpoint_mode"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VEBUS_ID): cv.use_id(VEBusHub),
        cv.Optional(CONF_VIRTUAL_MODE): switch.switch_schema(
            VEBusVirtualModeSwitch
        ).extend(cv.COMPONENT_SCHEMA),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VEBUS_ID])
    if CONF_VIRTUAL_MODE in config:
        conf = config[CONF_VIRTUAL_MODE]
        var = await switch.new_switch(conf)
        await cg.register_component(var, conf)
        cg.add(var.set_parent(hub))

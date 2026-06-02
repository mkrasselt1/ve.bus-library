import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@mkrasselt1"]
AUTO_LOAD = ["sensor", "binary_sensor"]
MULTI_CONF = False

vebus_ns = cg.esphome_ns.namespace("vebus")
VEBusHub = vebus_ns.class_("VEBusHub", cg.PollingComponent)

CONF_VEBUS_ID = "vebus_id"
CONF_RX_PIN = "rx_pin"
CONF_TX_PIN = "tx_pin"
CONF_DE_PIN = "de_pin"
CONF_CORE = "core"
CONF_INITIAL_ESS_POWER = "initial_ess_power"
CONF_VIRTUAL_MODE_INITIAL = "virtual_mode_initial"
CONF_VIRTUAL_MODE_DEADBAND = "virtual_mode_deadband"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(VEBusHub),
        cv.Required(CONF_RX_PIN): cv.int_,
        cv.Required(CONF_TX_PIN): cv.int_,
        cv.Required(CONF_DE_PIN): cv.int_,
        cv.Optional(CONF_CORE, default=0): cv.int_range(0, 1),
        cv.Optional(CONF_INITIAL_ESS_POWER, default=0): cv.int_range(-1875, 1875),
        cv.Optional(CONF_VIRTUAL_MODE_INITIAL, default=False): cv.boolean,
        cv.Optional(CONF_VIRTUAL_MODE_DEADBAND, default=10): cv.int_range(0, 1000),
    }
).extend(cv.polling_component_schema("1s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_rx_pin(config[CONF_RX_PIN]))
    cg.add(var.set_tx_pin(config[CONF_TX_PIN]))
    cg.add(var.set_de_pin(config[CONF_DE_PIN]))
    cg.add(var.set_core(config[CONF_CORE]))
    cg.add(var.set_initial_ess_power(config[CONF_INITIAL_ESS_POWER]))
    cg.add(var.set_virtual_mode_initial(config[CONF_VIRTUAL_MODE_INITIAL]))
    cg.add(var.set_virtual_mode_deadband(config[CONF_VIRTUAL_MODE_DEADBAND]))
    # The VEBus driver is bundled as VEBusDriver.{h,cpp} inside this
    # component so no separate library dependency is needed.

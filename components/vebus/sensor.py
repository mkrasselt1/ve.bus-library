import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)

from . import CONF_VEBUS_ID, VEBusHub

CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_DC_CURRENT = "dc_current"
CONF_TEMPERATURE = "temperature"
CONF_AC_POWER = "ac_power"
CONF_MAINS_VOLTAGE = "mains_voltage"
CONF_MAINS_CURRENT = "mains_current"
CONF_INVERTER_VOLTAGE = "inverter_voltage"
CONF_INVERTER_CURRENT = "inverter_current"
CONF_OUTPUT_POWER = "output_power"
CONF_MAINS_POWER = "mains_power"
CONF_SOC = "soc"
CONF_EFFECTIVE_ESS_POWER = "effective_ess_power"
CONF_AC_OUT_LOAD = "ac_out_load"


def _sens(unit, dev_class, accuracy=2):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        accuracy_decimals=accuracy,
        device_class=dev_class,
        state_class=STATE_CLASS_MEASUREMENT,
    )


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VEBUS_ID): cv.use_id(VEBusHub),
        cv.Optional(CONF_BATTERY_VOLTAGE): _sens(UNIT_VOLT, DEVICE_CLASS_VOLTAGE),
        cv.Optional(CONF_DC_CURRENT): _sens(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 1),
        cv.Optional(CONF_TEMPERATURE): _sens(UNIT_CELSIUS, DEVICE_CLASS_TEMPERATURE, 1),
        cv.Optional(CONF_AC_POWER): _sens(UNIT_WATT, DEVICE_CLASS_POWER, 0),
        cv.Optional(CONF_MAINS_VOLTAGE): _sens(UNIT_VOLT, DEVICE_CLASS_VOLTAGE),
        cv.Optional(CONF_MAINS_CURRENT): _sens(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 1),
        cv.Optional(CONF_INVERTER_VOLTAGE): _sens(UNIT_VOLT, DEVICE_CLASS_VOLTAGE),
        cv.Optional(CONF_INVERTER_CURRENT): _sens(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 1),
        cv.Optional(CONF_OUTPUT_POWER): _sens(UNIT_WATT, DEVICE_CLASS_POWER, 0),
        cv.Optional(CONF_MAINS_POWER): _sens(UNIT_WATT, DEVICE_CLASS_POWER, 0),
        cv.Optional(CONF_SOC): _sens(UNIT_PERCENT, DEVICE_CLASS_BATTERY, 1),
        cv.Optional(CONF_EFFECTIVE_ESS_POWER): _sens(UNIT_WATT, DEVICE_CLASS_POWER, 0),
        cv.Optional(CONF_AC_OUT_LOAD): _sens(UNIT_WATT, DEVICE_CLASS_POWER, 0),
    }
)


_SETTERS = [
    (CONF_BATTERY_VOLTAGE, "set_battery_voltage_sensor"),
    (CONF_DC_CURRENT, "set_dc_current_sensor"),
    (CONF_TEMPERATURE, "set_temperature_sensor"),
    (CONF_AC_POWER, "set_ac_power_sensor"),
    (CONF_MAINS_VOLTAGE, "set_mains_voltage_sensor"),
    (CONF_MAINS_CURRENT, "set_mains_current_sensor"),
    (CONF_INVERTER_VOLTAGE, "set_inverter_voltage_sensor"),
    (CONF_INVERTER_CURRENT, "set_inverter_current_sensor"),
    (CONF_OUTPUT_POWER, "set_output_power_sensor"),
    (CONF_MAINS_POWER, "set_mains_power_sensor"),
    (CONF_SOC, "set_soc_sensor"),
    (CONF_EFFECTIVE_ESS_POWER, "set_effective_ess_power_sensor"),
    (CONF_AC_OUT_LOAD, "set_ac_out_load_sensor"),
]


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VEBUS_ID])
    for key, setter in _SETTERS:
        if key in config:
            s = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, setter)(s))

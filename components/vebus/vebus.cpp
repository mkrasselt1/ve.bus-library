#include "vebus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace vebus {

static const char *const TAG = "vebus";

void VEBusHub::setup() {
  ESP_LOGCONFIG(TAG, "Setting up VE.Bus driver...");
  vebus_.begin(rx_pin_, tx_pin_, de_pin_, core_);
  vebus_.enableVirtualSetpointMode(virtual_mode_initial_, virtual_mode_deadband_);
  if (initial_ess_power_ != 0 || virtual_mode_initial_) {
    set_ess_power(initial_ess_power_);
  }
}

void VEBusHub::dump_config() {
  ESP_LOGCONFIG(TAG, "VE.Bus Hub:");
  ESP_LOGCONFIG(TAG, "  RX pin: GPIO%d", rx_pin_);
  ESP_LOGCONFIG(TAG, "  TX pin: GPIO%d", tx_pin_);
  ESP_LOGCONFIG(TAG, "  DE pin: GPIO%d", de_pin_);
  ESP_LOGCONFIG(TAG, "  Driver task core: %d", core_);
  ESP_LOGCONFIG(TAG, "  Virtual mode initial: %s (deadband %d W)",
                ONOFF(virtual_mode_initial_), virtual_mode_deadband_);
  LOG_UPDATE_INTERVAL(this);
}

void VEBusHub::update() {
  vebus_.requestReadRAM();
  if ((update_counter_ % 2) == 0) {
    const uint8_t ids[] = {
        VEBUS_RAM_UMAINS_RMS, VEBUS_RAM_IMAINS_RMS,
        VEBUS_RAM_UINVERTER_RMS, VEBUS_RAM_IINVERTER_RMS,
        VEBUS_RAM_OUTPUT_POWER, VEBUS_RAM_MAINS_POWER};
    vebus_.readRAMVars(ids, 6);
  } else if ((update_counter_ % 4) == 3) {
    const uint8_t ids[] = {VEBUS_RAM_CHARGE_STATE};
    vebus_.readRAMVars(ids, 1);
  }
  update_counter_++;

  if (battery_voltage_sensor_) battery_voltage_sensor_->publish_state(vebus_.getBatVolt());
  if (ac_power_sensor_)        ac_power_sensor_->publish_state(vebus_.getACPower());
  if (dc_current_sensor_)      dc_current_sensor_->publish_state(vebus_.getDCCurrent());
  if (temperature_sensor_)     temperature_sensor_->publish_state(vebus_.getTemp());

  if (vebus_.hasRAMVarResponse()) {
    uint8_t count = vebus_.getRAMVarCount();
    for (uint8_t i = 0; i < count; i++) {
      int16_t raw = vebus_.getRAMVarValue(i);
      switch (vebus_.getRAMVarId(i)) {
        case VEBUS_RAM_UMAINS_RMS:
          if (mains_voltage_sensor_) mains_voltage_sensor_->publish_state(raw * 0.01f);
          break;
        case VEBUS_RAM_IMAINS_RMS:
          if (mains_current_sensor_) mains_current_sensor_->publish_state(raw * 0.1f);
          break;
        case VEBUS_RAM_UINVERTER_RMS:
          if (inverter_voltage_sensor_) inverter_voltage_sensor_->publish_state(raw * 0.01f);
          break;
        case VEBUS_RAM_IINVERTER_RMS:
          if (inverter_current_sensor_) inverter_current_sensor_->publish_state(raw * 0.1f);
          break;
        case VEBUS_RAM_OUTPUT_POWER:
          if (output_power_sensor_) output_power_sensor_->publish_state(raw);
          vebus_.setACOutLoad(raw);
          break;
        case VEBUS_RAM_MAINS_POWER:
          if (mains_power_sensor_) mains_power_sensor_->publish_state(raw);
          break;
        case VEBUS_RAM_CHARGE_STATE:
          if (soc_sensor_) soc_sensor_->publish_state(raw * 0.5f);
          break;
      }
    }
    vebus_.clearRAMVarResponse();
  }

  if (effective_ess_power_sensor_)
    effective_ess_power_sensor_->publish_state(vebus_.getEffectiveESSPower());
  if (ac_out_load_sensor_)
    ac_out_load_sensor_->publish_state(vebus_.getACOutLoad());

  if (sync_bs_)  sync_bs_->publish_state(!vebus_.hasNoSync());
  if (dc_ok_bs_) dc_ok_bs_->publish_state(vebus_.dcLevelAllowsInverting());

  if (vebus_.hasNoSync()) {
    vebus_.requestWakeup();
  }
}

void VEBusHub::set_ess_power(int16_t w) {
  ess_power_ = w;
  vebus_.setESSPower(w);
}

void VEBusHub::enable_virtual_mode(bool en) {
  vebus_.enableVirtualSetpointMode(en, virtual_mode_deadband_);
}

}  // namespace vebus
}  // namespace esphome

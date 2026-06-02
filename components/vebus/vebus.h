#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

// VEBusDriver.{h,cpp} are a verbatim copy of the library's src/VEBus.{h,cpp}
// bundled inside this component so ESPHome compiles against the same code as
// the PlatformIO library — without needing a separate registry dependency.
// Keep them in sync if you edit src/VEBus.*.
#include "VEBusDriver.h"

namespace esphome {
namespace vebus {

class VEBusHub : public PollingComponent {
 public:
  VEBusHub() : PollingComponent(1000) {}

  // ---- Config setters ----
  void set_rx_pin(int p) { rx_pin_ = p; }
  void set_tx_pin(int p) { tx_pin_ = p; }
  void set_de_pin(int p) { de_pin_ = p; }
  void set_core(int c)   { core_  = c; }
  void set_initial_ess_power(int16_t w)     { initial_ess_power_ = w; }
  void set_virtual_mode_initial(bool b)     { virtual_mode_initial_ = b; }
  void set_virtual_mode_deadband(int16_t d) { virtual_mode_deadband_ = d; }

  // ---- Sensor setters (called from sensor.py codegen) ----
  void set_battery_voltage_sensor(sensor::Sensor *s)     { battery_voltage_sensor_ = s; }
  void set_dc_current_sensor(sensor::Sensor *s)          { dc_current_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s)         { temperature_sensor_ = s; }
  void set_ac_power_sensor(sensor::Sensor *s)            { ac_power_sensor_ = s; }
  void set_mains_voltage_sensor(sensor::Sensor *s)       { mains_voltage_sensor_ = s; }
  void set_mains_current_sensor(sensor::Sensor *s)       { mains_current_sensor_ = s; }
  void set_inverter_voltage_sensor(sensor::Sensor *s)    { inverter_voltage_sensor_ = s; }
  void set_inverter_current_sensor(sensor::Sensor *s)    { inverter_current_sensor_ = s; }
  void set_output_power_sensor(sensor::Sensor *s)        { output_power_sensor_ = s; }
  void set_mains_power_sensor(sensor::Sensor *s)         { mains_power_sensor_ = s; }
  void set_soc_sensor(sensor::Sensor *s)                 { soc_sensor_ = s; }
  void set_effective_ess_power_sensor(sensor::Sensor *s) { effective_ess_power_sensor_ = s; }
  void set_ac_out_load_sensor(sensor::Sensor *s)         { ac_out_load_sensor_ = s; }
  void set_sync_binary_sensor(binary_sensor::BinarySensor *s) { sync_bs_ = s; }
  void set_dc_ok_binary_sensor(binary_sensor::BinarySensor *s) { dc_ok_bs_ = s; }

  // ---- Component lifecycle ----
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // ---- Called by child entities ----
  void set_ess_power(int16_t w);
  int16_t get_ess_power() const { return ess_power_; }
  void enable_virtual_mode(bool en);
  bool is_virtual_mode() const { return vebus_.isVirtualSetpointMode(); }
  VEBus *driver() { return &vebus_; }

 protected:
  int rx_pin_{-1}, tx_pin_{-1}, de_pin_{-1}, core_{0};
  int16_t initial_ess_power_{0};
  bool    virtual_mode_initial_{false};
  int16_t virtual_mode_deadband_{10};
  int16_t ess_power_{0};

  VEBus    vebus_;
  uint32_t update_counter_{0};

  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *dc_current_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *ac_power_sensor_{nullptr};
  sensor::Sensor *mains_voltage_sensor_{nullptr};
  sensor::Sensor *mains_current_sensor_{nullptr};
  sensor::Sensor *inverter_voltage_sensor_{nullptr};
  sensor::Sensor *inverter_current_sensor_{nullptr};
  sensor::Sensor *output_power_sensor_{nullptr};
  sensor::Sensor *mains_power_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *effective_ess_power_sensor_{nullptr};
  sensor::Sensor *ac_out_load_sensor_{nullptr};
  binary_sensor::BinarySensor *sync_bs_{nullptr};
  binary_sensor::BinarySensor *dc_ok_bs_{nullptr};
};

}  // namespace vebus
}  // namespace esphome

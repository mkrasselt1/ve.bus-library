#include "vebus_number.h"

namespace esphome {
namespace vebus {

void VEBusESSPowerNumber::setup() {
  if (parent_ != nullptr)
    publish_state(static_cast<float>(parent_->get_ess_power()));
}

void VEBusESSPowerNumber::control(float value) {
  int16_t w = static_cast<int16_t>(value);
  if (parent_ != nullptr)
    parent_->set_ess_power(w);
  publish_state(value);
}

}  // namespace vebus
}  // namespace esphome

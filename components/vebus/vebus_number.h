#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "vebus.h"

namespace esphome {
namespace vebus {

class VEBusESSPowerNumber : public number::Number, public Component {
 public:
  void set_parent(VEBusHub *parent) { parent_ = parent; }
  void setup() override;

 protected:
  void control(float value) override;
  VEBusHub *parent_{nullptr};
};

}  // namespace vebus
}  // namespace esphome

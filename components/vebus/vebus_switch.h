#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "vebus.h"

namespace esphome {
namespace vebus {

class VEBusVirtualModeSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(VEBusHub *parent) { parent_ = parent; }
  void setup() override {
    if (parent_ != nullptr)
      publish_state(parent_->is_virtual_mode());
  }

 protected:
  void write_state(bool state) override {
    if (parent_ != nullptr)
      parent_->enable_virtual_mode(state);
    publish_state(state);
  }
  VEBusHub *parent_{nullptr};
};

}  // namespace vebus
}  // namespace esphome

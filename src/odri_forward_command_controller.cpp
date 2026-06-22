// Copyright 2024 LAAS-CNRS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "odri_forward_command_controller/odri_forward_command_controller.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace odri_forward_command_controller {

static constexpr const char* HW_IF_GAIN_KP = "gain_kp";
static constexpr const char* HW_IF_GAIN_KD = "gain_kd";

OdriForwardCommandController::OdriForwardCommandController()
    : controller_interface::ControllerInterface(),
      joints_command_subscriber_(nullptr) {}

controller_interface::CallbackReturn OdriForwardCommandController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s\n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OdriForwardCommandController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();
  joint_names_ = params_.joints;

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  pos_interface_names_.clear();
  vel_interface_names_.clear();
  eff_interface_names_.clear();
  kp_interface_names_.clear();
  kd_interface_names_.clear();

  for (const auto& joint : joint_names_) {
    pos_interface_names_.push_back(joint + "/" +
                                   hardware_interface::HW_IF_POSITION);
    vel_interface_names_.push_back(joint + "/" +
                                   hardware_interface::HW_IF_VELOCITY);
    eff_interface_names_.push_back(joint + "/" +
                                   hardware_interface::HW_IF_EFFORT);
    kp_interface_names_.push_back(joint + "/" + HW_IF_GAIN_KP);
    kd_interface_names_.push_back(joint + "/" + HW_IF_GAIN_KD);
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { rt_command_.set(*msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
OdriForwardCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.insert(config.names.end(), pos_interface_names_.begin(),
                      pos_interface_names_.end());
  config.names.insert(config.names.end(), vel_interface_names_.begin(),
                      vel_interface_names_.end());
  config.names.insert(config.names.end(), eff_interface_names_.begin(),
                      eff_interface_names_.end());
  config.names.insert(config.names.end(), kp_interface_names_.begin(),
                      kp_interface_names_.end());
  config.names.insert(config.names.end(), kd_interface_names_.begin(),
                      kd_interface_names_.end());
  return config;
}

controller_interface::InterfaceConfiguration
OdriForwardCommandController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn OdriForwardCommandController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  const std::size_t n = joint_names_.size();

  auto assign_interfaces = [&](const std::vector<std::string>& names,
                               std::vector<LoanedRef>& out) -> bool {
    std::vector<LoanedRef> tmp;
    if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, names, std::string(""), tmp) ||
        tmp.size() != n) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Expected %zu interfaces for '%s', got %zu", n,
                   names.empty()
                       ? "?"
                       : names[0].substr(names[0].rfind('/') + 1).c_str(),
                   tmp.size());
      return false;
    }
    out = std::move(tmp);
    return true;
  };

  if (!assign_interfaces(pos_interface_names_, pos_interfaces_) ||
      !assign_interfaces(vel_interface_names_, vel_interfaces_) ||
      !assign_interfaces(eff_interface_names_, eff_interfaces_) ||
      !assign_interfaces(kp_interface_names_, kp_interfaces_) ||
      !assign_interfaces(kd_interface_names_, kd_interfaces_)) {
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_commands_ = CmdType{};
  rt_command_.set(joint_commands_);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OdriForwardCommandController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  pos_interfaces_.clear();
  vel_interfaces_.clear();
  eff_interfaces_.clear();
  kp_interfaces_.clear();
  kd_interfaces_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OdriForwardCommandController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  auto cmd_op = rt_command_.try_get();
  if (cmd_op.has_value()) {
    joint_commands_ = cmd_op.value();
  }

  const std::size_t n = joint_names_.size();
  const auto& data = joint_commands_.data;

  // Require exactly 5*n values: [pos×n | vel×n | eff×n | gain_kp×n | gain_kd×n]
  if (data.size() != 5 * n) {
    return controller_interface::return_type::OK;
  }

  auto apply = [&](std::size_t offset, std::vector<LoanedRef>& ifaces) {
    for (std::size_t i = 0; i < n; ++i) {
      if (std::isfinite(data[offset + i]) &&
          !ifaces[i].get().set_value(data[offset + i])) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                             *(get_node()->get_clock()), 1000,
                             "Failed to set command interface '%s'",
                             ifaces[i].get().get_name().c_str());
      }
    }
  };

  apply(0 * n, pos_interfaces_);
  apply(1 * n, vel_interfaces_);
  apply(2 * n, eff_interfaces_);
  apply(3 * n, kp_interfaces_);
  apply(4 * n, kd_interfaces_);

  return controller_interface::return_type::OK;
}

}  // namespace odri_forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    odri_forward_command_controller::OdriForwardCommandController,
    controller_interface::ControllerInterface)

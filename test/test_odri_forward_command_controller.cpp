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

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/test_utils.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "odri_forward_command_controller/odri_forward_command_controller.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using controller_interface::activate_succeeds;
using controller_interface::configure_succeeds;
using controller_interface::deactivate_succeeds;
using hardware_interface::CommandInterface;
using hardware_interface::LoanedCommandInterface;

static constexpr const char * HW_IF_GAIN_KP = "gain_kp";
static constexpr const char * HW_IF_GAIN_KD = "gain_kd";

// Re-exports protected members as public so tests can inject commands and inspect state.
class FriendController : public odri_forward_command_controller::OdriForwardCommandController
{
public:
  using OdriForwardCommandController::rt_command_;
  using OdriForwardCommandController::pos_interfaces_;
  using OdriForwardCommandController::vel_interfaces_;
  using OdriForwardCommandController::eff_interfaces_;
  using OdriForwardCommandController::kp_interfaces_;
  using OdriForwardCommandController::kd_interfaces_;
};

class OdriForwardCommandControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() { controller_ = std::make_unique<FriendController>(); }
  void TearDown() { controller_.reset(); }

  // Initialise controller and wire up all 5 command interfaces per joint.
  // joints is passed via node_options so generate_parameter_library sees it at init time.
  void SetUpController(const std::vector<std::string> & joints = {"j1", "j2"})
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "odri_forward_command_controller";
    params.robot_description = "";
    params.controller_manager_update_rate = 100;
    params.node_namespace = "";
    auto node_opts = controller_->define_custom_node_options();
    node_opts.append_parameter_override("joints", joints);
    params.node_options = node_opts;
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);

    std::vector<LoanedCommandInterface> cmd_ifs;
    cmd_ifs.emplace_back(j1_pos_);
    cmd_ifs.emplace_back(j1_vel_);
    cmd_ifs.emplace_back(j1_eff_);
    cmd_ifs.emplace_back(j1_kp_);
    cmd_ifs.emplace_back(j1_kd_);
    cmd_ifs.emplace_back(j2_pos_);
    cmd_ifs.emplace_back(j2_vel_);
    cmd_ifs.emplace_back(j2_eff_);
    cmd_ifs.emplace_back(j2_kp_);
    cmd_ifs.emplace_back(j2_kd_);
    controller_->assign_interfaces(std::move(cmd_ifs), {});
    executor_.add_node(controller_->get_node()->get_node_base_interface());
  }

protected:
  std::unique_ptr<FriendController> controller_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // backing storage — two joints
  double j1_pos_val_{0.0}, j1_vel_val_{0.0}, j1_eff_val_{0.0}, j1_kp_val_{0.0}, j1_kd_val_{0.0};
  double j2_pos_val_{0.0}, j2_vel_val_{0.0}, j2_eff_val_{0.0}, j2_kp_val_{0.0}, j2_kd_val_{0.0};

  // Named CommandInterface objects (LoanedCommandInterface requires non-const lvalue ref)
  CommandInterface j1_pos_{"j1", hardware_interface::HW_IF_POSITION, &j1_pos_val_};
  CommandInterface j1_vel_{"j1", hardware_interface::HW_IF_VELOCITY, &j1_vel_val_};
  CommandInterface j1_eff_{"j1", hardware_interface::HW_IF_EFFORT, &j1_eff_val_};
  CommandInterface j1_kp_{"j1", HW_IF_GAIN_KP, &j1_kp_val_};
  CommandInterface j1_kd_{"j1", HW_IF_GAIN_KD, &j1_kd_val_};

  CommandInterface j2_pos_{"j2", hardware_interface::HW_IF_POSITION, &j2_pos_val_};
  CommandInterface j2_vel_{"j2", hardware_interface::HW_IF_VELOCITY, &j2_vel_val_};
  CommandInterface j2_eff_{"j2", hardware_interface::HW_IF_EFFORT, &j2_eff_val_};
  CommandInterface j2_kp_{"j2", HW_IF_GAIN_KP, &j2_kp_val_};
  CommandInterface j2_kd_{"j2", HW_IF_GAIN_KD, &j2_kd_val_};
};

// ---------------------------------------------------------------------------

TEST_F(OdriForwardCommandControllerTest, JointsParamMissing)
{
  // Init with empty joints (default) — configure must fail
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "odri_forward_command_controller";
  params.robot_description = "";
  params.controller_manager_update_rate = 100;
  params.node_namespace = "";
  params.node_options = controller_->define_custom_node_options();
  ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
  EXPECT_FALSE(configure_succeeds(controller_));
}

TEST_F(OdriForwardCommandControllerTest, ConfigureSuccess)
{
  SetUpController();
  EXPECT_TRUE(configure_succeeds(controller_));
}

TEST_F(OdriForwardCommandControllerTest, ActivateSuccess)
{
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  EXPECT_TRUE(activate_succeeds(controller_));
}

TEST_F(OdriForwardCommandControllerTest, ActivateWrongJointFails)
{
  // Controller configured for j1/j_wrong, but hardware only exposes j1/j2.
  // on_activate returns ERROR → lifecycle transitions to unconfigured/finalized,
  // so activate_succeeds throws rather than returning false.
  SetUpController({"j1", "j_wrong"});
  ASSERT_TRUE(configure_succeeds(controller_));
  EXPECT_THROW(activate_succeeds(controller_), std::runtime_error);
}

TEST_F(OdriForwardCommandControllerTest, CommandForwardedCorrectly)
{
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  // data layout: [pos×2 | vel×2 | eff×2 | kp×2 | kd×2]
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0, 2.0,    // positions  j1, j2
              0.1, 0.2,    // velocities j1, j2
              10.0, 20.0,  // efforts    j1, j2
              5.0, 6.0,    // gains_kp   j1, j2
              0.5, 0.6};   // gains_kd   j1, j2

  controller_->rt_command_.set(cmd);
  EXPECT_EQ(
    controller_->update(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(j1_pos_val_, 1.0);
  EXPECT_DOUBLE_EQ(j2_pos_val_, 2.0);
  EXPECT_DOUBLE_EQ(j1_vel_val_, 0.1);
  EXPECT_DOUBLE_EQ(j2_vel_val_, 0.2);
  EXPECT_DOUBLE_EQ(j1_eff_val_, 10.0);
  EXPECT_DOUBLE_EQ(j2_eff_val_, 20.0);
  EXPECT_DOUBLE_EQ(j1_kp_val_, 5.0);
  EXPECT_DOUBLE_EQ(j2_kp_val_, 6.0);
  EXPECT_DOUBLE_EQ(j1_kd_val_, 0.5);
  EXPECT_DOUBLE_EQ(j2_kd_val_, 0.6);
}

TEST_F(OdriForwardCommandControllerTest, NaNSkipped)
{
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  j1_pos_val_ = 99.0;
  j2_pos_val_ = 99.0;
  j1_kp_val_ = 99.0;
  j2_kp_val_ = 99.0;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  std_msgs::msg::Float64MultiArray cmd;
  // pos j1=NaN (untouched), j2=3.0; kp j1=7.0, j2=8.0; vel/eff/kd zeros
  cmd.data = {nan, 3.0,   // positions
              0.0, 0.0,   // velocities
              0.0, 0.0,   // efforts
              7.0, 8.0,   // gains_kp
              0.0, 0.0};  // gains_kd

  controller_->rt_command_.set(cmd);
  controller_->update(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));

  EXPECT_DOUBLE_EQ(j1_pos_val_, 99.0);  // NaN → untouched
  EXPECT_DOUBLE_EQ(j2_pos_val_, 3.0);
  EXPECT_DOUBLE_EQ(j1_kp_val_, 7.0);
  EXPECT_DOUBLE_EQ(j2_kp_val_, 8.0);
}

TEST_F(OdriForwardCommandControllerTest, WrongSizeIgnored)
{
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  j1_pos_val_ = 55.0;
  j2_pos_val_ = 66.0;

  // Only 4 values instead of 10 — entire message must be ignored
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0, 2.0, 3.0, 4.0};

  controller_->rt_command_.set(cmd);
  controller_->update(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));

  EXPECT_DOUBLE_EQ(j1_pos_val_, 55.0);  // untouched
  EXPECT_DOUBLE_EQ(j2_pos_val_, 66.0);  // untouched
}

TEST_F(OdriForwardCommandControllerTest, DeactivateClearsInterfaces)
{
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_TRUE(deactivate_succeeds(controller_));

  EXPECT_TRUE(controller_->pos_interfaces_.empty());
  EXPECT_TRUE(controller_->vel_interfaces_.empty());
  EXPECT_TRUE(controller_->eff_interfaces_.empty());
  EXPECT_TRUE(controller_->kp_interfaces_.empty());
  EXPECT_TRUE(controller_->kd_interfaces_.empty());
}

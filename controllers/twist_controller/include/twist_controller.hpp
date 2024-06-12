// Copyright 2023 Kristófi Mihály
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


#ifndef KUKA_CONTROLLERS__TWIST_CONTROLLER_HPP_
#define KUKA_CONTROLLERS__TWIST_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include "forward_command_controller/multi_interface_forward_command_controller.hpp"

#include "visibility_control.h"
#include "twist_controller_parameters.hpp"


#include "geometry_msgs/msg/twist.hpp"

namespace kuka_controllers
{
class TwistController : public forward_command_controller::ForwardControllersBase
{
public:
  TWIST_CONTROLLER_PUBLIC TwistController();
  TWIST_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

private:
  TWIST_CONTROLLER_LOCAL void declare_parameters() override;
  TWIST_CONTROLLER_LOCAL controller_interface::CallbackReturn read_parameters()
    override;

  using Params = twist_controller::Params;
  using ParamListener = twist_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_command_subscriber_;
  geometry_msgs::msg::Twist last_command_msg_;
  std_msgs::msg::Float64MultiArray command_;
  std::shared_ptr<forward_command_controller::CmdType> command_ptr_;
};

}  // namespace kuka_controllers
#endif  // KUKA_CONTROLLERS__TWIST_CONTROLLER_HPP_

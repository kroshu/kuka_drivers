// Copyright 2024 Mihaly Kristofi
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

#ifndef WRENCH_CONTROLLER__WRENCH_CONTROLLER_HPP_
#define WRENCH_CONTROLLER__WRENCH_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "forward_command_controller/multi_interface_forward_command_controller.hpp"

#include "wrench_controller/visibility_control.h"
#include "wrench_controller_parameters.hpp"

namespace kuka_controllers
{
class WrenchController : public forward_command_controller::ForwardControllersBase
{
public:
  WRENCH_CONTROLLER_PUBLIC WrenchController();

private:
  WRENCH_CONTROLLER_LOCAL void declare_parameters() override;
  WRENCH_CONTROLLER_LOCAL controller_interface::CallbackReturn read_parameters() override;

  using Params = wrench_controller::Params;
  using ParamListener = wrench_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};
}  // namespace kuka_controllers
#endif  // WRENCH_CONTROLLER__WRENCH_CONTROLLER_HPP_

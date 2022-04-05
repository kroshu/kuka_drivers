// Copyright 2020 Zoltán Rési
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

#include "robot_control/joint_controller.hpp"

#include <sys/mman.h>
#include <string>
#include <memory>
#include <vector>

namespace robot_control
{

double d2r(double degrees)
{
  return degrees / 180 * M_PI;
}

JointController::JointController(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options)
{
  parameter_set_access_rights_.emplace(
    "max_velocities_degPs",
    ParameterSetAccessRights {true, true, false, false});
  parameter_set_access_rights_.emplace(
    "lower_limits_deg",
    ParameterSetAccessRights {true, true, false, false});
  parameter_set_access_rights_.emplace(
    "upper_limits_deg",
    ParameterSetAccessRights {true, true, false, false});
  parameter_set_access_rights_.emplace(
    "velocity_scaling",
    ParameterSetAccessRights {true, true, false, false});

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();

  reference_joint_state_listener_ = this->create_subscription<
    sensor_msgs::msg::JointState>(
    "reference_joint_state", qos,
    [this](sensor_msgs::msg::JointState::SharedPtr state) {
      this->referenceUpdateCallback(state);
    } /*, sub_opt*/);

  measured_joint_state_listener_ = this->create_subscription<
    sensor_msgs::msg::JointState>(
    "measured_joint_state", qos,
    [this](sensor_msgs::msg::JointState::SharedPtr state) {
      this->jointStateMeasurementsCallback(state);
    } /*, sub_opt*/);

  joint_command_publisher_ = this->create_publisher<
    sensor_msgs::msg::JointState>("joint_command", qos);
  joint_controller_is_active_publisher_ = this->create_publisher<
    std_msgs::msg::Bool>("joint_controller_is_active", qos);


  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->onParamChange(parameters);
    });

  this->declare_parameter(
    "max_velocities_degPs",
    rclcpp::ParameterValue(
      std::vector<double>(
        {300, 300, 400, 300, 160,
          160, 400})));

  this->declare_parameter(
    "lower_limits_deg",
    rclcpp::ParameterValue(
      std::vector<double>(
        {-170, -120, -170,
          -120, -170, -120, -175})));
  this->declare_parameter(
    "upper_limits_deg",
    rclcpp::ParameterValue(
      std::vector<double>(
        {170, 120, 170, 120,
          170, 120, 175})));

  this->declare_parameter(
    "velocity_scaling",
    rclcpp::ParameterValue(velocity_scaling_));

  auto set_rate_callback = [this](
    kuka_sunrise_interfaces::srv::SetDouble::Request::SharedPtr request,
    kuka_sunrise_interfaces::srv::SetDouble::Response::SharedPtr response) {
      int cmd_per_frame = static_cast<int>(JointController::ms_in_sec_ /
        loop_period_ms_ / (8 * request->data)) + 1;
      if (cmd_per_frame > 2) {
        cmd_per_frame_temp_ = cmd_per_frame;
        RCLCPP_INFO(
          get_logger(),
          "Successfully changed rate, receiving commands in every %i. frame",
          cmd_per_frame);
        response->success = true;
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Control loop frequency should be bigger than command receive frequency");
        response->success = false;
      }
    };

  set_rate_service_ = this->create_service<kuka_sunrise_interfaces::srv::SetDouble>(
    "joint_controller/set_rate", set_rate_callback);

  auto send_period_callback = [this](
    kuka_sunrise_interfaces::srv::SetInt::Request::SharedPtr request,
    kuka_sunrise_interfaces::srv::SetInt::Response::SharedPtr response) {
      if (this->get_current_state().label() != "active") {
        send_period_ms_ = request->data;
        loop_period_ms_ = send_period_ms_ * receive_multiplier_;
        updateMaxPositionDifference();
        RCLCPP_INFO(
          get_logger(),
          "Succesfully synced send period");
        response->success = true;
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Joint controller is active, could not change send_period");
        response->success = false;
      }
    };

  sync_send_period_service_ = this->create_service<
    kuka_sunrise_interfaces::srv::SetInt>("sync_send_period", send_period_callback);

  auto receive_multiplier_callback = [this](
    kuka_sunrise_interfaces::srv::SetInt::Request::SharedPtr request,
    kuka_sunrise_interfaces::srv::SetInt::Response::SharedPtr response) {
      if (this->get_current_state().label() != "active") {
        receive_multiplier_ = request->data;
        loop_period_ms_ = send_period_ms_ * receive_multiplier_;
        updateMaxPositionDifference();
        RCLCPP_INFO(
          get_logger(),
          "Succesfully synced receive multiplier");
        response->success = true;
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Joint controller is active, could not change receive_multiplier");
        response->success = false;
      }
    };

  sync_receive_multiplier_service_ = this->create_service<
    kuka_sunrise_interfaces::srv::SetInt>("sync_receive_multiplier", receive_multiplier_callback);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointController::
on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_controller_is_active_ = std::make_shared<std_msgs::msg::Bool>();

  joint_command_ = std::make_shared<sensor_msgs::msg::JointState>();
  joint_command_->position.resize(7);
  joint_command_->velocity.resize(7);
  joint_command_->effort.resize(7);
  // TODO(Zoltan Resi) change stack size with setrlimit rlimit_stack?
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    RCLCPP_ERROR(get_logger(), "mlockall error");
    RCLCPP_ERROR(get_logger(), strerror(errno));
    return ERROR;
  }

  struct sched_param param;
  param.sched_priority = 90;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    RCLCPP_ERROR(get_logger(), "setscheduler error");
    RCLCPP_ERROR(get_logger(), strerror(errno));
    return ERROR;
  }
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointController::
on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (munlockall() == -1) {
    RCLCPP_ERROR(get_logger(), "munlockall error");
    return ERROR;
  }
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointController::
on_activate(
  const rclcpp_lifecycle::State &)
{
  joint_command_publisher_->on_activate();
  joint_controller_is_active_publisher_->on_activate();
  joint_controller_is_active_->data = true;
  joint_controller_is_active_publisher_->publish(
    *joint_controller_is_active_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointController::
on_deactivate(
  const rclcpp_lifecycle::State &)
{
  joint_command_publisher_->on_deactivate();
  joint_controller_is_active_->data = false;
  joint_controller_is_active_publisher_->publish(
    *joint_controller_is_active_);
  joint_controller_is_active_publisher_->on_deactivate();
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointController::
on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result =
    SUCCESS;
  switch (state.id()) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      result = this->on_deactivate(get_current_state());
      if (result != SUCCESS) {
        break;
      }
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;
    default:
      break;
  }
  return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointController::on_error(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "Lifecycle management caught an error");
  return SUCCESS;
}

const std::vector<double> & JointController::maxVelocitiesRadPs() const
{
  return max_velocities_radPs_;
}

const std::vector<double> & JointController::lowerLimitsRad() const
{
  return lower_limits_rad_;
}

const std::vector<double> & JointController::upperLimitsRad() const
{
  return upper_limits_rad_;
}

rcl_interfaces::msg::SetParametersResult JointController::onParamChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter & param : parameters) {
    if (param.get_name() == "max_velocities_degPs" && canSetParameter(param)) {
      result.successful = onMaxVelocitiesChangeRequest(param);
    } else if (param.get_name() == "lower_limits_deg" &&  // NOLINT
      canSetParameter(param))
    {
      result.successful = onLowerLimitsChangeRequest(param);
    } else if (param.get_name() == "upper_limits_deg" &&  // NOLINT
      canSetParameter(param))
    {
      result.successful = onUpperLimitsChangeRequest(param);
    } else if (param.get_name() == "velocity_scaling" &&  // NOLINT
      canSetParameter(param))
    {
      result.successful = onVelocityScalingChangeRequest(param);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Invalid parameter name %s",
        param.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

bool JointController::canSetParameter(const rclcpp::Parameter & param)
{
  try {
    if (!parameter_set_access_rights_.at(param.get_name()).isSetAllowed(
        this->get_current_state().id()))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Parameter %s cannot be changed while in state %s",
        param.get_name().c_str(),
        this->get_current_state().label().c_str());
      return false;
    }
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Parameter set access rights for parameter %s couldn't be determined",
      param.get_name().c_str());
    return false;
  }
  return true;
}

bool JointController::onMaxVelocitiesChangeRequest(
  const rclcpp::Parameter & param)
{
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }

  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(), param.as_double_array().end(),
    max_velocities_radPs_.begin(), [](double v) {
      return d2r(v * 0.9);
    });
  updateMaxPositionDifference();
  return true;
}

bool JointController::onLowerLimitsChangeRequest(
  const rclcpp::Parameter & param)
{
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }

  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(),
    param.as_double_array().end(), lower_limits_rad_.begin(), [](double v) {
      return d2r(v * 0.9);
    });
  return true;
}

bool JointController::onUpperLimitsChangeRequest(
  const rclcpp::Parameter & param)
{
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }

  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(),
    param.as_double_array().end(), upper_limits_rad_.begin(), [](double v) {
      return d2r(v * 0.9);
    });
  return true;
}

bool JointController::onVelocityScalingChangeRequest(
  const rclcpp::Parameter & param)
{
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (!canSetParameter(param)) {
    return false;
  }
  velocity_scaling_ = param.as_bool();
  return true;
}

void JointController::updateMaxPositionDifference()
{
  auto calc_pos_diff = [ & loop_period_ms_ = loop_period_ms_](double v) {
      return v * loop_period_ms_ / JointController::ms_in_sec_;
    };
  std::transform(
    max_velocities_radPs_.begin(), max_velocities_radPs_.end(),
    max_position_difference_.begin(), calc_pos_diff);
}

void JointController::jointStateMeasurementsCallback(
  sensor_msgs::msg::JointState::SharedPtr measured_joint_state)
{
  if (++loop_count_ == receive_multiplier_) {
    loop_count_ = 0;
    controlLoopCallback(measured_joint_state);
  }
}

void JointController::controlLoopCallback(
  sensor_msgs::msg::JointState::SharedPtr measured_joint_state)
{
  if (!reference_joint_state_) {
    reference_joint_state_ = measured_joint_state;
  }
  if (reference_joint_state_->position.size() == 7) {
    if (velocity_scaling_) {
      setJointCommandPositionWithVelocity(measured_joint_state->position);
    } else {
      joint_command_->position = reference_joint_state_->position;
    }
    enforceSpeedLimits(measured_joint_state->position);
    cmd_count_++;
  }
  if (reference_joint_state_->velocity.size() == 7) {
    joint_command_->velocity = reference_joint_state_->velocity;
  }
  if (reference_joint_state_->effort.size() == 7) {
    joint_command_->effort = reference_joint_state_->effort;
  }
  joint_command_->header = measured_joint_state->header;
  joint_command_publisher_->publish(*joint_command_);
}

// TODO(kovacsge11) right now it's ok this way,
// we would need to change other functions, too,
// but in general would be nicer if we could separate it into a child class
void JointController::setJointCommandPositionWithVelocity(
  const std::vector<double> & measured_joint_position)
{
  const std::vector<double> & reference_joint_position =
    reference_joint_state_->position;
  std::vector<double> & joint_command_position = joint_command_->position;
  for (int i = 0; i < 7; i++) {
    if (max_position_difference_[i] < 0) {
      RCLCPP_WARN(get_logger(), "max position difference is not positive");
    }
    double position_error = reference_joint_position[i] -
      measured_joint_position[i];
    double reference_error = reference_joint_position[i] -
      joint_command_position[i];

    // Set speed so, that the motion finishes when the next reference is received
    if (cmd_count_ >= JointController::cmd_per_frame_) {
      joint_command_position[i] = reference_joint_position[i];
      if (i == 0) {
        RCLCPP_DEBUG(
          get_logger(),
          "Frame not received in expected time, command count is %i",
          cmd_count_);
      }
    } else if (start_flag_) {  // First command: based on measured
      joint_command_position[i] = measured_joint_position[i] +
        position_error / (JointController::cmd_per_frame_ - cmd_count_);
      if (i == 6) {
        start_flag_ = false;
        prev_ref_joint_pos_ = reference_joint_position;
      }
      RCLCPP_DEBUG(get_logger(), "First command");
    } else {  // Not first command: based on previous command
      joint_command_position[i] += reference_error /
        (JointController::cmd_per_frame_ - cmd_count_);
      RCLCPP_DEBUG(get_logger(), "Command calculated relative to previous");
    }
  }
}

void JointController::enforceSpeedLimits(
  const std::vector<double> & measured_joint_position)
{
  std::vector<double> & joint_command_position = joint_command_->position;
  for (int i = 0; i < 7; i++) {
    // If axis is marked, slow down motion at new reference
    double vel_factor;
    if (slow_start_[i]) {
      if (cmd_count_ == 0) {
        vel_factor = 0.55;
      } else if (cmd_count_ == 1) {
        vel_factor = 0.8;
      } else {
        vel_factor = 1;
      }
    } else {
      vel_factor = 1;
    }
    if (abs(measured_joint_position[i] - joint_command_position[i]) <=
      max_position_difference_[i] * vel_factor)
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Successfully set step size to the speed of movement");
    } else if (joint_command_position[i] > measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] +
        max_position_difference_[i] * vel_factor;
      RCLCPP_DEBUG(get_logger(), "Movement was too fast around joint %i", i + 1);

    } else if (joint_command_position[i] < measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] -
        max_position_difference_[i] * vel_factor;
      RCLCPP_DEBUG(get_logger(), "Movement was too fast around joint %i", i + 1);

    } else {
      RCLCPP_ERROR(get_logger(), "Reference or measured joint state is NaN");
    }
  }
}

void JointController::referenceUpdateCallback(
  sensor_msgs::msg::JointState::SharedPtr reference_joint_state)
{
  if (this->get_current_state().label() != "active") {
    return;
  }
  auto & reference_joint_positions = reference_joint_state->position;
  if (!reference_joint_state_) {
    reference_joint_state_ = reference_joint_state;
  }
  auto & p_reference_joint_positions = reference_joint_state_->position;
  for (int i = 0; i < 7; i++) {
    if (reference_joint_positions[i] < lower_limits_rad_[i]) {
      reference_joint_positions[i] = lower_limits_rad_[i];
      RCLCPP_WARN(
        get_logger(),
        "Reference for joint %i exceeded lower limit", i + 1);
    } else if (reference_joint_positions[i] > upper_limits_rad_[i]) {
      reference_joint_positions[i] = upper_limits_rad_[i];
      RCLCPP_WARN(
        get_logger(),
        "Reference for joint %i exceeded upper limit", i + 1);
    }
    // if the change of reference changes sign, mark that axis to be slowed down
    if ((p_reference_joint_positions[i] - prev_ref_joint_pos_[i]) *
      (reference_joint_positions[i] - p_reference_joint_positions[i]) > 0)
    {
      slow_start_[i] = false;
    } else if (reference_joint_positions[i] == p_reference_joint_positions[i]) {
      slow_start_[i] = false;
    } else if (reference_joint_positions[i] > p_reference_joint_positions[i] &&  // NOLINT
      joint_command_->position[i] > reference_joint_positions[i])
    {
      slow_start_[i] = false;
    } else if (reference_joint_positions[i] < p_reference_joint_positions[i] &&  // NOLINT
      joint_command_->position[i] < reference_joint_positions[i])
    {
      slow_start_[i] = false;
    } else {
      slow_start_[i] = true;
    }
  }
  RCLCPP_DEBUG(get_logger(), "commands per frame: %i", cmd_count_);
  cmd_count_ = 0;
  if (cmd_per_frame_temp_) {
    cmd_per_frame_ = cmd_per_frame_temp_;
    cmd_per_frame_temp_ = 0;
  }
  prev_ref_joint_pos_ = reference_joint_state_->position;
  reference_joint_state_ = reference_joint_state;
}

}  // namespace robot_control

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<robot_control::JointController>(
    "joint_controller", rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

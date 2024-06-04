# Core functionalities for the drivers

This package contains two libraries that implement the common functionalities of the 3 kuka drivers to reduce code duplications and make the code more maintainable.

## Wrapper methods for specific services
The `kuka_drivers_core::communication_helpers` is a header-only library providing wrapper methods for commonly used features.

#### Synchronous service calls

`rclcpp` does not provide synchronous service calls, this is implemented in the [`service_tools.hpp`](https://github.com/kroshu/kuka_drivers/blob/master/kuka_drivers_core/include/communication_helpers/service_tools.hpp)

It provides the `sendRequest()` endpoint with following arguments:
- client [ClientT]: the initialized service client
- request [RequestT]: the filled request with appropriate type
- service_timeout_ms [int] (default: 2000): timeout for service discovery
- response_timeout_ms [int] (default: 1000): timeout for getting the response

The method returns the service response (as a shared pointer).

Example for calling the `ListControllers` service of the `controller_manager`:
```C++
rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr get_controllers_client_;
auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

// [...]

auto response =  kuka_drivers_core::sendRequest<controller_manager_msgs::srv::ListControllers::Response>(get_controllers_client_, request, 0, 1000);
```

#### `ros2_control` state handling

The library also contains the [`ros2_control_tools.hpp`](https://github.com/kroshu/kuka_drivers/blob/master/kuka_drivers_core/include/communication_helpers/ros2_control_tools.hpp) header, which implements wrapper methods for modifying the states of controllers and hardware components.

**Endpoints:**

The `changeHardwareState()` can change the state of one hardware component and has the following arguments:
- `client` [rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr]: initialized client
- hardware_name` [std::string]: name of the hardware component
- `state` [uint8_t of [enum](https://docs.ros2.org/foxy/api/lifecycle_msgs/msg/State.html)]: desired state after state change (only one transition is possible with one call)
- `timeout_ms` [int] (default: 1000): timeout for the response

The method returns whether the transition was successful.

The `changeControllerState()` can change the state of more controllers and has the following arguments:
- `client` [rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr]: initialized client
- `activate_controllers` [std::vector\<std::string\>]: names of the controllers to activate
- `deactivate_controllers` [std::vector\<std::string\>]: names of the controllers to deactivate
- `strictness` [int32_t] (default: STRICT): whether to fail if one state change is unsuccessful

The method returns whether the transitions were successful.


Examples:
```C++
rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr change_hardware_state_client_;
rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr change_controller_state_client_;

// [...]

// Activate hardware named 'lbr_iisy3_r760'
bool success1 = changeHardwareState(change_hardware_state_client_, "lbr_iisy3_r760", State::PRIMARY_STATE_ACTIVE);

// Activate 'joint_state_broadcaster' and 'joint_trajectory_controller'
bool success2 = changeControllerState(change_controller_state_client_, {"joint_state_broadcaster", "joint_trajectory_controller"}, {/*nothing to deactivate*/});
```

## Core classes

### Base classes with improved parameter handling

There are two core classes implemented in this repository, `ROS2BaseNode` for improved parameter handling, and `ROS2BaseLCNode`, which derives from the `rclcpp_lifecycle::LifecycleNode` class and implements lifecycle state transitions which would be usually implemented in the same way in every case. These are virtual functions, so it is possible to override them in the case of a different desired implementation.

#### Parameter handling

The base classes provide a wrapper method for parameter registration, which makes handling of parameters more convenient.
This is done with the help of the `ParameterHandler` class, which includes a `ParameterBase` and a template `Parameter<T>` nested class for this purpose.

Improvements:
- The `add_on_set_parameters_callback()` is called in both of the constructors to register the callback for parameter change requests. This makes sure that the initial values of the parameters are synced from the parameter server.
- The parameter type is enforced automatically.
- The registered callback has access over all of the registered parameters, therefore the parameter server and the node is always in sync
- It is easy to register a callback for additional checks before requested parameter value is accepted.
- The user can define the lifecycle states, in which parameter changes are allowed using the `ParameterSetAccessRights` structure.
- There is a different endpoint for static parameters, which cannot be changed after initialization.

The `Parameter<T>` class supports the following parameter types (identical to the types supported by `rclcpp::Parameter`):
 - bool
 - int64_t (or type satisfying `std::is_integral` except bool)
 - double (or type satisfying `std::is_floating_point`)
 - std::string
 - std::vector\<uint8_t\>
 - std::vector\<bool\>
 - std::vector\<int64_t\>
 - std::vector\<double\>
 - std::vector\<std::string\>


The nodes provide the `registerParameter()` and `registerStaticParameter()` endpoints with the following arguments:
 - `name` [std::string]: name of the parameter
 - `value` [T]: default value of the parameter
 - `rights` [ParameterSetAccessRights]: structure defining whether modifying the parameter value is allowed in `inactive` and `active` states (only for `ROS2BaseLCNode`, value changes are always allowed in `unconfigured` state)
 - `on_change_callback` [std::function<bool(const T &)>]: the callback to call when determining the validity of a parameter change request

Both methods register a parameter with the given `name` and `type`, and the `on_change_callback` is called if a parameter change is requested to check validity. In case of the `registerStaticParameter()`, the callback always returns false after initializing the value.

Example code for registering an integer parameter for both base nodes (`onRateChangeRequest()` checks the validity of the requested rate and returns a boolean):
```C++
  // For class derived from ROS2BaseLCNode
  registerParameter<int>(
    "rate", 2, kuka_drivers_core::ParameterSetAccessRights {true, false},
      [this](int rate) {
      return this->onRateChangeRequest(rate);
    });

  // For class derived from ROS2BaseNode
  registerParameter<int>(
    "rate", 2, [this](int rate) {
      return this->onRateChangeRequest(rate);
    });
```

To modify the callback that is called for validating every parameter change, one has to remove the registered callback and add the new one. For example to disable all parameter changes if a *parameter_change_blocked_* flag is set, it is possible to add the condition to the registered callback:

```C++
  remove_on_set_parameters_callback(ParamCallback().get());
  ParamCallback() = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      if (!parameter_change_blocked_) {
        return getParameterHandler().onParamChange(parameters);
      } else {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        return result;
      }
    });
```

## Control mode handler

The package also contains the `ControllerHandler` class, which is responsible for tracking the active controllers and on control mode changes return the controller names that need to be activated and deactivated based on the control mode definitions.

There is a defined set of controllers that must be active for every control mode, the `GetControllersForSwitch()` method determines the switches necessary in case of a control mode change. The method returns the names of the controllers to be switched, to make this possible, the names of the controllers should be provided for every controller type with the `UpdateControllerName()` method.

If the switch was successful, the `ApproveControllerActivation()` and `ApproveControllerDeactivation()` methods should be called to update the internal state of the `ControllerHandler` class.

The class can also handle controllers that should be active in every control mode (e.g. `joint_state_broadcaster`), these should be be given in the constructor as the `fixed_controllers` argument (std::vector).

## Type definitions and modified control node

Additionally common type definitions are included for control modes (see details on the [wiki](https://github.com/kroshu/kuka_drivers/wiki#control-mode-definitions)) and hardware interface types.

The package also contains the [modified `control_node`](https://github.com/kroshu/kuka_drivers/wiki#real-time-interface) that instantiates the `controller_manager` without managing the timing.

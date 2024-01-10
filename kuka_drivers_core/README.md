# Core functionalities for the drivers

This package contains two libraries that implement the common functionalities of the 3 kuka drivers.

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

Endpoints:
The `changeHardwareState()` can change the state of one hardware component and has the following arguments:
- `client` [rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr]: initialized client
- hardware_name` [std::string]: name of the hardware component
- `state` [uint8_t of [enum](https://docs.ros2.org/foxy/api/lifecycle_msgs/msg/State.html)]: desired state after state change (only one transition is possible with one call)
- `timeout_ms` [int] (default: 1000): timeout for the response
The method returns whether the transition was successul.

The `changeControllerState()` can change the state of more controllers and has the following arguments:
- `client` [rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr]: initialized client
- `activate_controllers` [std::vector<std::string>]: names of the controllers to activate
- `deactivate_controllers` [std::vector<std::string>]: names of the controllers to deactivate
- `strictness` [int32_t] (default: STRICT): whether to fail if one state change is unsuccessful
The method returns whether the transitions were successul.


Examples:
```C++
rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr change_hardware_state_client_;
rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr change_controller_state_client_;

// [...]

// Activate hardware named 'lbr_iisy_r760'
bool success1 = changeHardwareState(change_hardware_state_client_, "lbr_iisy_r760", State::PRIMARY_STATE_ACTIVE);

// Activate 'joint_state_broadcaster' and 'joint_trajectory_controller'
bool success2 = changeControllerState(change_controller_state_client_, {"joint_state_broadcaster", "joint_trajectory_controller"}, {/*nothing to deactivate*/});
```

## Core classes which help function the repositories of kroshu.
These classes provide functionalities which are frequently used in ROS2 environment.
Deriving from these classes the user has a helpful wrapper around the base functionalities of ROS2.

Right now there are two classes implemented in this repository, ROS2BaseNode for simple parameter handling, and ROS2BaseLCNode, which additionally furthers the rclcpp_lifecycle::LifecycleNode class by implementing lifecycle functions which would be usually implemented in the same way in every case. These are virtual functions, so it is possible to override them in the case of a different desired implementation.

The parameter handling is better designed, than the one provided by the rclcpp::Parameter class.
This is done with the help of the ParameterHandler class, which includes a ParameterBase and a template Parameter<T> nested class for this purpose. They are extended with the member functions of the ParameterHandler class, which handle all the node's parameters and the related issues with the help of a heterogeneous collection.

One can use these base classes by deriving from one of them.
To declare a parameter and manage its changes, the registerParameter\<T\> template function must be used with the following arguments:
 - name of parameter (std::string)
 - default value of Parameter (type T)
 - ParameterAccessRights structure defining in which states is the setting of the Parameter allowed (only for ROS2BaseLCNode)
 - the callback to call when determining the validity of a parameter change request (std::function<bool(const T &)>)

Example code for registering an integer parameter for both base nodes (onRateChangeRequest() returns a boolean):
```C++
  // Derived from  ROS2BaseLCNode
  registerParameter<int>(
    "rate", 2, kuka_drivers_core::ParameterSetAccessRights {true, true,
      false, false}, [this](int rate) {
      return this->onRateChangeRequest(rate);
    });

  // Derived from  ROS2BaseNode
  registerParameter<int>(
    "rate", 2, [this](int rate) {
      return this->onRateChangeRequest(rate);
    });
```

The add_on_set_parameters_callback() is called in the base node constructors, always before the parameter declarations, so the initial values of the parameters will be always synced from the parameter server. To modify this callback (e.g. add a condition to all parameter change callbacks), one has to remove the registered callback and add the new one:

```C++
  remove_on_set_parameters_callback(ParamCallback().get());
  ParamCallback() = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      if (<condition>) {
        return getParameterHandler().onParamChange(parameters);
      } else {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        return result;
      }
    });
```

The template argument of the Parameter class should be one of the following (others result in a compile error):
 - bool
 - int64_t (or type satisfying std::is_integral except bool)
 - double (or type satisfying std::is_floating_point)
 - std::string
 - std::vector\<uint8_t\>
 - std::vector\<bool\>
 - std::vector\<int64_t\>
 - std::vector\<double\>
 - std::vector\<std::string\>

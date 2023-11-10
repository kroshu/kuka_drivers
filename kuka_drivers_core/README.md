Github CI | SonarCloud
------------| ---------------
[![Build Status](https://github.com/kroshu/kuka_drivers_core/workflows/CI/badge.svg?branch=master)](https://github.com/kroshu/kuka_drivers_core/actions) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kroshu_kuka_drivers_core&metric=alert_status)](https://sonarcloud.io/dashboard?id=kroshu_kuka_drivers_core)

# Core classes which help function the repositories of kroshu.
These classes provide functonalities which are frequently used in ROS2 environment.
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

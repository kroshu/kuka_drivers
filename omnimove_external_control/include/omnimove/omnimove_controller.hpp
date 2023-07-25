#ifndef OMNIMOVE_CONTROLLER_HPP
#define OMNIMOVE_CONTROLLER_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>

namespace omnimove{
    class OmnimoveController:public hardware_interface::SystemInterface{
    public:
        OmnimoveController();
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&) override;


    };
}
#endif // OMNIMOVE_CONTROLLER_HPP

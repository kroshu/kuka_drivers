#include "omnimove/omnimove_external_control.hpp"
using namespace hardware_interface;
using namespace rclcpp_lifecycle::node_interfaces;
namespace  omnimove{

    OmnimoveExternalControl::OmnimoveExternalControl():hardware_interface::SystemInterface(){
    }
    LifecycleNodeInterface::CallbackReturn OmnimoveExternalControl::on_init(const hardware_interface::HardwareInfo& info){
        if (SystemInterface::on_init(info)!= LifecycleNodeInterface::CallbackReturn::SUCCESS){
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    }

    hardware_interface::CallbackReturn OmnimoveExternalControl::on_configure(const rclcpp_lifecycle::State & previous_state){
        return SystemInterface::on_configure(previous_state);
    }

    hardware_interface::CallbackReturn OmnimoveExternalControl::on_cleanup(const rclcpp_lifecycle::State &previous_state){
        return SystemInterface::on_cleanup(previous_state);
    }

    hardware_interface::CallbackReturn OmnimoveExternalControl::on_shutdown(const rclcpp_lifecycle::State& previous_state){
        return SystemInterface::on_shutdown(previous_state);
    }

    hardware_interface::CallbackReturn OmnimoveExternalControl::on_error(const rclcpp_lifecycle::State &previous_state){
        return SystemInterface::on_error(previous_state);
    }

    std::vector<StateInterface> OmnimoveExternalControl::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interface;
        state_interface.emplace_back("move", "position", &position_state[0]);
        state_interface.emplace_back("move", "velocity", &velocity_state[0]);

        return state_interface;
    }

    std::vector<CommandInterface> OmnimoveExternalControl::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interface;
        command_interface.emplace_back("move", "velocity", &velocity_commands[0]);
        return command_interface;
    }

    hardware_interface::return_type OmnimoveExternalControl::read(const rclcpp::Time&, const rclcpp::Duration&){
        return return_type::OK;
    }

    hardware_interface::return_type OmnimoveExternalControl::write(const rclcpp::Time&, const rclcpp::Duration&){
        return return_type::OK;
    }

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        omnimove::OmnimoveExternalControl, hardware_interface::SystemInterface)

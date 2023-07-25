#include "omnimove/omnimove_controller.hpp"
using namespace hardware_interface;
using namespace rclcpp_lifecycle::node_interfaces;
namespace  omnimove{

    OmnimoveController::OmnimoveController():hardware_interface::SystemInterface(){
    }
    LifecycleNodeInterface::CallbackReturn OmnimoveController::on_init(const hardware_interface::HardwareInfo& info){
        if (SystemInterface::on_init(info)!= LifecycleNodeInterface::CallbackReturn::SUCCESS){
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    }

    hardware_interface::CallbackReturn OmnimoveController::on_configure(const rclcpp_lifecycle::State & previous_state){
        return SystemInterface::on_configure(previous_state);
    }

    hardware_interface::CallbackReturn OmnimoveController::on_cleanup(const rclcpp_lifecycle::State &previous_state){
        return SystemInterface::on_cleanup(previous_state);
    }

    hardware_interface::CallbackReturn OmnimoveController::on_shutdown(const rclcpp_lifecycle::State& previous_state){
        return SystemInterface::on_shutdown(previous_state);
    }

    hardware_interface::CallbackReturn OmnimoveController::on_error(const rclcpp_lifecycle::State &previous_state){
        return SystemInterface::on_error(previous_state);
    }

    std::vector<StateInterface> OmnimoveController::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interface;
        return state_interface;
    }

    std::vector<CommandInterface> OmnimoveController::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interface;
        return command_interface;
    }

    hardware_interface::return_type OmnimoveController::read(const rclcpp::Time & time, const rclcpp::Duration & period){
        return return_type::OK;
    }

    hardware_interface::return_type OmnimoveController::write(const rclcpp::Time & time, const rclcpp::Duration & period){
        return return_type::OK;
    }

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        omnimove::OmnimoveController, hardware_interface::SystemInterface)

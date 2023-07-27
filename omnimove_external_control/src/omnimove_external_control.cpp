#include "omnimove/omnimove_external_control.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
using namespace hardware_interface;
using namespace rclcpp_lifecycle::node_interfaces;
using namespace std;
namespace  omnimove{

    OmnimoveExternalControl::OmnimoveExternalControl():hardware_interface::SystemInterface(){
    }
    LifecycleNodeInterface::CallbackReturn OmnimoveExternalControl::on_init(const hardware_interface::HardwareInfo& info){
        if (SystemInterface::on_init(info)!= LifecycleNodeInterface::CallbackReturn::SUCCESS){
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        position_state_.resize(info.joints.size());
        velocity_state_.resize(info.joints.size());
        velocity_commands_.resize(info.joints.size());

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
        for(unsigned int i=0; i<info_.joints.size(); ++i){
            state_interface.emplace_back(info_.joints[i].name, "position", &position_state_[i]);
            state_interface.emplace_back(info_.joints[i].name, "velocity", &velocity_state_[i]);
        }
        return state_interface;
    }

    std::vector<CommandInterface> OmnimoveExternalControl::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interface;
        for(unsigned int i=0; i<info_.joints.size(); ++i){
            command_interface.emplace_back(info_.joints[i].name, "velocity", &velocity_commands_[i]);
        }

        return command_interface;
    }

    hardware_interface::return_type OmnimoveExternalControl::read(const rclcpp::Time&, const rclcpp::Duration&){
        return return_type::OK;
    }

    hardware_interface::return_type OmnimoveExternalControl::write(const rclcpp::Time&, const rclcpp::Duration&){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("OmnimoveExternalControl"), "writing "<< velocity_commands_[0]
                <<" "<<velocity_commands_[1]<<" "<<velocity_commands_[2]);
        return return_type::OK;
    }

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        omnimove::OmnimoveExternalControl, hardware_interface::SystemInterface)

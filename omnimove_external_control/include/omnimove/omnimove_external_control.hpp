#ifndef OMNIMOVE_CONTROLLER_HPP
#define OMNIMOVE_CONTROLLER_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <omnimove/external_control_message.hpp>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <rcl/logging.h>

namespace omnimove{
    class OmnimoveExternalControl:public hardware_interface::SystemInterface{
    public:
        OmnimoveExternalControl();
        ~OmnimoveExternalControl();
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&) override;
        hardware_interface::CallbackReturn on_activate (const rclcpp_lifecycle::State &) override;
     private:
        std::vector<double> velocity_state_;
        std::vector<double> position_state_;
        std::vector<double> velocity_commands_;
        std::vector<double> position_commands_;
        std::string protocol_version_;
        int external_control_port_;
        size_t velocity_x_index_, velocity_y_index_, velocity_theta_index_;
        size_t velocity_pillar1_index_, velocity_pillar2_index_, velocity_pillar3_index_, velocity_pillar4_index_, velocity_blade_index_;
        size_t position_pillar1_index_, position_pillar2_index_, position_pillar3_index_, position_pillar4_index_, position_blade_index_;

        boost::asio::io_context io_context_;
        std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
        std::unique_ptr<boost::asio::ip::tcp::socket> client_socket_;
        boost::circular_buffer<uint8_t> read_buffer_;
        ExternalControlData parseLastMessageFromBuffer();



    };
}
#endif // OMNIMOVE_CONTROLLER_HPP

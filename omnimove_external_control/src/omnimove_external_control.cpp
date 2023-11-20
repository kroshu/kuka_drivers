#include "omnimove/omnimove_external_control.hpp"
#include "omnimove/external_control_message.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <boost/circular_buffer.hpp>
using namespace hardware_interface;
using namespace rclcpp_lifecycle::node_interfaces;
using namespace boost::asio::ip;
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
        protocol_version_ = std::stoi(info_.hardware_parameters["protocol_version"]); //should be 1.6
        external_control_port_ = std::stoi(info_.hardware_parameters["port"]);
        RCLCPP_INFO(
                    rclcpp::get_logger("OmnimoveExternalControl"),
                    "port of external control server:%d", external_control_port_);

        //        client_socket_ = std::make_shared<tcp::socket>(io_service_);
        //      client_socket_->connect(tcp::endpoint(address::from_string(external_control_host_), external_control_port_));

        acceptor_ = std::make_unique<tcp::acceptor>(io_context_, tcp::v4(), external_control_port_);
        RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "Waiting for External Control Connection");
        //can i wait for a maximum amount of time
        client_socket_ = tcp::socket(io_context_);
        acceptor_->accept(client_socket_);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    }

    hardware_interface::CallbackReturn OmnimoveExternalControl::on_configure(const rclcpp_lifecycle::State & previous_state){
        return SystemInterface::on_configure(previous_state);
    }

    hardware_interface::CallbackReturn OmnimoveExternalControl::on_cleanup(const rclcpp_lifecycle::State &previous_state){
        return SystemInterface::on_cleanup(previous_state);
    }

    hardware_interface::CallbackReturn OmnimoveExternalControl::on_shutdown(const rclcpp_lifecycle::State& previous_state){
        //TODO: need to stop listening here

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


    ExternalControlData OmnimoveExternalControl::parseLastMessageFromBuffer(){
        //assume that sequence will start from the beginning
        const int expected_data_size = 96;
        size_t last_message_start = (read_buffer_.size()/expected_data_size -1) *expected_data_size;
        for (size_t i=0 ; i < last_message_start; ++i){
            read_buffer_.pop_front();
        }
        //now let's check the header.
        const char* expected_header = ExternalControlData::EXTERNAL_CONTROL_DATA_HEADER;
        for(size_t i = 0; i < strlen(expected_header); ++i){
            if (read_buffer_.at (i)!=expected_header[i]){
                RCLCPP_ERROR(rclcpp::get_logger("OmnimoveExternalControl"), "header does not match");
                return ExternalControlData();
            }
        }
        //now we will connect the latest message

        ExternalControlData received_data(read_buffer_.);
        //TODO: need to check CRC
    }
    hardware_interface::return_type OmnimoveExternalControl::read(const rclcpp::Time&, const rclcpp::Duration&){

        std::array<char, 1024> buffer;
        size_t bytes_received = client_socket_.receive(buffer);
        read_buffer_.insert(read_buffer_.end (), buffer.begin(), buffer.begin() + bytes_received);
        // parse all messages from the buffer.
        // incomplete messages need to be kept for further reading.
        ExternalControlData readData = parseLastMessageFromBuffer();
        if (readData.isValid()){
            velocity_state_[0] = readData.speedX();
            velocity_state_[1] = readData.speedY();
            velocity_state_[2] = readData.speedW();
        }
        return return_type::OK;
    }


    hardware_interface::return_type OmnimoveExternalControl::write(const rclcpp::Time&, const rclcpp::Duration&){
        /**  RCLCPP_INFO_STREAM(rclcpp::get_logger("OmnimoveExternalControl"), "writing "<< velocity_commands_[0]
                <<" "<<velocity_commands_[1]<<" "<<velocity_commands_[2]);*/
        client_socket_.send(ExternalControlOmnimoveDriveCommand(velocity_commands_[1],
                            velocity_commands_[1],
                velocity_commands_[2]).getSerialisedData());
        return return_type::OK;
    }

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        omnimove::OmnimoveExternalControl, hardware_interface::SystemInterface)

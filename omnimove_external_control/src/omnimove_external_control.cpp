#include "omnimove/omnimove_external_control.hpp"
#include "omnimove/external_control_message.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/array.hpp>

using namespace hardware_interface;
using namespace rclcpp_lifecycle::node_interfaces;
using namespace boost::asio::ip;
using namespace std;
namespace  omnimove{

    OmnimoveExternalControl::OmnimoveExternalControl():hardware_interface::SystemInterface(), read_buffer_(4096){
        rclcpp::get_logger("OmnimoveExternalControl") =  rclcpp::get_logger ("OmnimoveExternalControl");
    }



    LifecycleNodeInterface::CallbackReturn OmnimoveExternalControl::on_init(const hardware_interface::HardwareInfo& info){
        if (SystemInterface::on_init(info)!= LifecycleNodeInterface::CallbackReturn::SUCCESS){
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        position_state_.resize(info.joints.size());
        velocity_state_.resize(info.joints.size());
        velocity_commands_.resize(info.joints.size());
        protocol_version_ = info_.hardware_parameters["protocol_version"]; //should be 1.6
        external_control_port_ = std::stoi(info_.hardware_parameters["port"]);
        RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"),
                    "port of external control server:%d", external_control_port_);
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
        //client_socket_->shutdown ();
        return SystemInterface::on_shutdown(previous_state);
    }

    CallbackReturn OmnimoveExternalControl::on_activate (const rclcpp_lifecycle::State &previous_state){
        boost::asio::io_context io_context;

        //        client_socket_ = std::make_shared<tcp::socket>(io_service_);
        //      client_socket_->connect(tcp::endpoint(address::from_string(external_control_host_), external_control_port_));
        try{
            acceptor_.reset(new tcp::acceptor(io_context_, tcp::endpoint(tcp::v4(), external_control_port_)));
            //tcp::acceptor *accepter = new tcp::acceptor(io_context, tcp::endpoint(tcp::v4(), external_control_port_));
        }catch (std::exception &e){
            RCLCPP_ERROR (rclcpp::get_logger("OmnimoveExternalControl"),  "Failed to start acceptor %s %d", e.what(), external_control_port_);
            return CallbackReturn::ERROR;

        }

        RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "Waiting for External Control Connection");
        //can i wait for a maximum amount of time
        try{
            client_socket_ = std::make_unique<tcp::socket>(io_context);
        }catch(std::exception &e){
            RCLCPP_ERROR (rclcpp::get_logger("OmnimoveExternalControl"),  "Failed to initialise socket %s", e.what());
            return CallbackReturn::ERROR;
        }
        try{
            acceptor_->accept(*client_socket_.get());
        } catch (std::exception &e){
            RCLCPP_ERROR (rclcpp::get_logger("OmnimoveExternalControl"),  "Failed to accept connection %s", e.what());
            return CallbackReturn::ERROR;
        }
        return SystemInterface::on_activate(previous_state);
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
        if (last_message_start > read_buffer_.size ()){
            RCLCPP_ERROR(rclcpp::get_logger("OmnimoveExternalControl"),
                         "Message too small last_message_start %lu, read_buffer_size %lu", last_message_start, read_buffer_.size ());
            return ExternalControlData();
        }
        RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"),
                     "last_message_start %lu, read_buffer_size %lu", last_message_start, read_buffer_.size ());

        for (size_t i=0 ; i < last_message_start; ++i){
            read_buffer_.pop_front();
        }
        //now let's check the header.
        if (!ExternalControlData::isMessageValid(read_buffer_)){
            const char* EXTERNAL_CONTROL_DATA_HEADER = "KMRUTV03";

            RCLCPP_ERROR(rclcpp::get_logger("OmnimoveExternalControl"),
                         "header does not match %lu", strlen(EXTERNAL_CONTROL_DATA_HEADER));
            for (size_t i = 0; i < strlen(EXTERNAL_CONTROL_DATA_HEADER); ++i) {
              if (read_buffer_[i] != EXTERNAL_CONTROL_DATA_HEADER[i]) {
                  RCLCPP_ERROR(rclcpp::get_logger("OmnimoveExternalControl"), "msg_data is %d, header is %d", read_buffer_[i], EXTERNAL_CONTROL_DATA_HEADER[i]);

              }
            }

            return ExternalControlData();

        }

        return ExternalControlData(read_buffer_
                                   );
        //now we will connect the latest message

        //ExternalControlData received_data(read_buffer_.);
    }
    hardware_interface::return_type OmnimoveExternalControl::read(const rclcpp::Time&, const rclcpp::Duration&){

        std::array<char, 1024> buffer;
        RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "Am waiting to read something if available");
        size_t bytes_received = client_socket_->read_some(boost::asio::buffer(buffer));
        RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "received %lu", bytes_received);

        read_buffer_.insert(read_buffer_.end (), buffer.begin(), buffer.begin() + bytes_received);
        RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "Inserted into buffer");

        // parse all messages from the buffer.
        // incomplete messages need to be kept for further reading.
        ExternalControlData readData = parseLastMessageFromBuffer();
        RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "Finished parsing into buffer");

        if (readData.isDataValid()){
            velocity_state_[0] = (double) readData.speedX();
            velocity_state_[1] = (double) readData.speedY();
            velocity_state_[2] = (double) readData.speedW();
        }
        return return_type::OK;
    }


    hardware_interface::return_type OmnimoveExternalControl::write(const rclcpp::Time&, const rclcpp::Duration&){
          RCLCPP_INFO_STREAM(rclcpp::get_logger("OmnimoveExternalControl"), "writing "<< velocity_commands_[0]
                <<" "<<velocity_commands_[1]<<" "<<velocity_commands_[2]);
        client_socket_->send(ExternalControlOmnimoveDriveCommand(velocity_commands_[1],
                            velocity_commands_[1],
                velocity_commands_[2]).getSerialisedData());
        return return_type::OK;
    }

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        omnimove::OmnimoveExternalControl, hardware_interface::SystemInterface)

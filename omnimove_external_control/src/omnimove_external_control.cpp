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

    OmnimoveExternalControl::OmnimoveExternalControl():hardware_interface::SystemInterface(),
        velocity_x_index_(-1),
        velocity_y_index_(-1),
        velocity_theta_index_(-1),
        velocity_pillar1_index_(-1),
        velocity_pillar2_index_(-1),
        velocity_pillar3_index_(-1),
        velocity_pillar4_index_(-1),
        velocity_blade_index_(-1),
        read_buffer_(4096){
        rclcpp::get_logger("OmnimoveExternalControl") =  rclcpp::get_logger ("OmnimoveExternalControl");
    }

    OmnimoveExternalControl::~OmnimoveExternalControl(){
        if (acceptor_->is_open ()){
            acceptor_->close ();
        }

        if (client_socket_->is_open ()){
           client_socket_->close ();
        }
    }

    LifecycleNodeInterface::CallbackReturn OmnimoveExternalControl::on_init(const hardware_interface::HardwareInfo& info){
        if (SystemInterface::on_init(info)!= LifecycleNodeInterface::CallbackReturn::SUCCESS){
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        int position_states_size = 0;
        int velocity_states_size = 0;
        int velocity_commands_size = 0;
        int position_commands_size = 0;
        for (size_t i=0; i < info.joints.size ();++i){
            auto command_interfaces = info.joints[i].command_interfaces;
            auto state_interfaces = info.joints[i].state_interfaces;
            for (auto ci:command_interfaces){
                std::string joint_name = info.joints[i].name;

                if (ci.name == "velocity"){
                    if (joint_name == "move_x"){
                        velocity_x_index_ = velocity_commands_size;
                    }else if (joint_name == "move_y"){
                        velocity_y_index_ = velocity_commands_size;
                    }else if (joint_name =="move_theta"){
                        velocity_theta_index_ = velocity_commands_size;
                    }else if (joint_name == "pillar1"){
                        velocity_pillar1_index_ = velocity_commands_size;
                    }else if (joint_name == "pillar2"){
                        velocity_pillar2_index_ = velocity_commands_size;
                    }else if (joint_name == "pillar3"){
                        velocity_pillar3_index_ = velocity_commands_size;
                    }else if (joint_name == "pillar4"){
                        velocity_pillar4_index_ = velocity_commands_size;
                    }else if (joint_name == "shield"){
                        velocity_blade_index_ = velocity_commands_size;
                    }
                    velocity_commands_size++;
                }
                if (ci.name == "position"){
                    if (joint_name == "pillar1"){
                        position_pillar1_index_ = position_commands_size;
                    }else if (joint_name == "pillar2"){
                        position_pillar2_index_ = position_commands_size;
                    }else if (joint_name == "pillar3"){
                        position_pillar3_index_ = position_commands_size;
                    }else if (joint_name == "pillar4"){
                        position_pillar4_index_ = position_commands_size;
                    }else if (joint_name == "shield"){
                        position_blade_index_ = position_commands_size;
                    }

                    position_commands_size++;
                }
            }
            for (auto si:state_interfaces){
                if (si.name == "position"){
                    position_states_size ++;
                }else if (si.name =="velocity"){
                    velocity_states_size++;
                }
            }
        }
        position_state_.resize(position_states_size);
        velocity_state_.resize(velocity_states_size);
        velocity_commands_.resize(velocity_commands_size);
        position_commands_.resize (position_commands_size);
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
        client_socket_->shutdown(client_socket_->shutdown_both);
        acceptor_->close();
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
        size_t position_states = 0;
        size_t velocity_states = 0;
        for (size_t i=0; i < info_.joints.size ();++i){
            auto state_interfaces = info_.joints[i].state_interfaces;
            for (auto si:state_interfaces){
                if (si.name == "position"){
                    state_interface.emplace_back(info_.joints[i].name, "position", &position_state_[position_states++]);
                }
                if (si.name == "velocity"){
                    state_interface.emplace_back(info_.joints[i].name, "velocity", &velocity_state_[velocity_states++]);
                }
            }
        }
        return state_interface;
    }


    std::vector<CommandInterface> OmnimoveExternalControl::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interface;
        size_t position_commands = 0;
        size_t velocity_commands = 0;
        for(unsigned int i=0; i<info_.joints.size(); ++i){
            auto command_interfaces = info_.joints[i].command_interfaces;
            for (auto ci: command_interfaces){
                if (ci.name == "velocity"){
                    command_interface.emplace_back(info_.joints[i].name, "velocity", &velocity_commands_[velocity_commands++]);
                }
                if (ci.name == "position"){
                    command_interface.emplace_back(info_.joints[i].name, "position", &position_commands_[position_commands++]);
                }
            }
        }

        return command_interface;
    }


    ExternalControlData OmnimoveExternalControl::parseLastMessageFromBuffer(){
        //assume that sequence will start from the beginning
        const int expected_data_size = ExternalControlData::totalMessageLength ();

        size_t last_message_start = (read_buffer_.size()/expected_data_size -1) *expected_data_size;
        if (last_message_start > read_buffer_.size ()){
            RCLCPP_ERROR(rclcpp::get_logger("OmnimoveExternalControl"),
                         "Message too small last_message_start %lu, read_buffer_size %lu", last_message_start, read_buffer_.size ());
            return ExternalControlData();
        }
//        RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"),
//                     "last_message_start %lu, read_buffer_size %lu", last_message_start, read_buffer_.size ());

        for (size_t i=0 ; i < last_message_start; ++i){
            read_buffer_.pop_front();
        }
        //now let's check the header.
        std::unique_ptr<char[]> msg_data = std::make_unique<char[]>(read_buffer_.size());
        for (size_t i=0; i < read_buffer_.size(); ++i){
            msg_data.get()[i] = read_buffer_[i];
        }
        if (!ExternalControlData::isMessageValid(msg_data.get(), read_buffer_.size ())){
            const char* EXTERNAL_CONTROL_DATA_HEADER = "KMRUTV03";

            for (size_t i = 0; i < strlen(EXTERNAL_CONTROL_DATA_HEADER); ++i) {
              if (read_buffer_[i] != EXTERNAL_CONTROL_DATA_HEADER[i]) {
                  RCLCPP_ERROR(rclcpp::get_logger("OmnimoveExternalControl"), "msg_data is %d, header is %d", read_buffer_[i], EXTERNAL_CONTROL_DATA_HEADER[i]);

              }
            }

            return ExternalControlData();

        }
        ExternalControlData parsedData(msg_data.get() + 8);
        read_buffer_.erase (read_buffer_.begin (), read_buffer_.begin ()+ expected_data_size);
        return parsedData;
        //now we will connect the latest message

        //ExternalControlData received_data(read_buffer_.);
    }
    hardware_interface::return_type OmnimoveExternalControl::read(const rclcpp::Time&, const rclcpp::Duration&){

        std::array<char, 1024> buffer;
     //   RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "Am waiting to read something if available");
        size_t bytes_received = client_socket_->read_some(boost::asio::buffer(buffer));
       // RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "received %lu", bytes_received);

        read_buffer_.insert(read_buffer_.end (), buffer.begin(), buffer.begin() + bytes_received);
       // RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "Inserted into buffer");

        // parse all messages from the buffer.
        // incomplete messages need to be kept for further reading.
        ExternalControlData readData = parseLastMessageFromBuffer();
      //  RCLCPP_INFO(rclcpp::get_logger("OmnimoveExternalControl"), "Finished parsing into buffer");

        if (readData.isDataValid()){
            velocity_state_[velocity_x_index_] = (double) readData.speedX();
            if (velocity_y_index_ < velocity_state_.size ()){
                velocity_state_[velocity_y_index_] = (double) readData.speedY();
            }
            velocity_state_[velocity_theta_index_] = (double) readData.speedW();
            if (velocity_blade_index_ < velocity_state_.size ()){
                velocity_state_[velocity_blade_index_] = (double) readData.speedShield();
            }
            if (velocity_pillar1_index_ < velocity_state_.size ()){
                velocity_state_[velocity_pillar1_index_] = (double) readData.speedPillar1 ();
            }

            if (velocity_pillar2_index_ < velocity_state_.size ()){
                velocity_state_[velocity_pillar2_index_] = (double) readData.speedPillar2 ();
            }
            if (velocity_pillar3_index_ < velocity_state_.size ()){
                velocity_state_[velocity_pillar3_index_] = (double) readData.speedPillar3 ();
            }
            if (velocity_pillar4_index_ < velocity_state_.size ()){
                velocity_state_[velocity_pillar4_index_] = (double) readData.speedPillar4 ();
            }

            if (position_pillar1_index_  <  position_state_.size()){
                position_state_[position_pillar1_index_] = (double) readData.posPillar1 ();
            }

            if (position_pillar2_index_  <  position_state_.size()){
                position_state_[position_pillar2_index_] = (double) readData.posPillar2 ();
            }

            if (position_pillar3_index_  <  position_state_.size()){
                position_state_[position_pillar3_index_] = (double) readData.posPillar3 ();
            }

            if (position_pillar4_index_  <  position_state_.size()){
                position_state_[position_pillar4_index_] = (double) readData.posPillar4 ();
            }

            if (position_blade_index_  <  position_state_.size()){
                position_state_[position_blade_index_] = (double) readData.posShield ();
            }

        }
        return return_type::OK;
    }


    hardware_interface::return_type OmnimoveExternalControl::write(const rclcpp::Time&, const rclcpp::Duration&){
        //  RCLCPP_INFO_STREAM(rclcpp::get_logger("OmnimoveExternalControl"), "writing "<< velocity_commands_[0]
          //      <<" "<<velocity_commands_[1]<<" "<<velocity_commands_[2]);
        client_socket_->send(ExternalControlOmnimoveDriveCommand(velocity_commands_[0],
                            velocity_commands_[1],
                velocity_commands_[2]).getSerialisedData());
        return return_type::OK;
    }

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        omnimove::OmnimoveExternalControl, hardware_interface::SystemInterface)

#include "kuka_utv3_ec_driver/kuka_utv3_hardware_interface.hpp"
#include <boost/array.hpp>
#include <boost/circular_buffer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "kuka_utv3_ec_driver/external_control_message.hpp"

using namespace hardware_interface;
using namespace rclcpp_lifecycle::node_interfaces;
using namespace boost::asio::ip;
using namespace std;
namespace omnimove
{

KukaUTV3HardwareInterface::KukaUTV3HardwareInterface()
: hardware_interface::SystemInterface(), read_buffer_(4096)
{
  rclcpp::get_logger("OmnimoveExternalControl") = rclcpp::get_logger("OmnimoveExternalControl");
}

KukaUTV3HardwareInterface::~KukaUTV3HardwareInterface()
{
  if (acceptor_->is_open())
  {
    acceptor_->close();
  }

  if (client_socket_->is_open())
  {
    client_socket_->close();
  }
}

LifecycleNodeInterface::CallbackReturn KukaUTV3HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  agv_type_ = info.hardware_parameters.at("agv_type");

  if (agv_type_ == "caterpillar")
  {
    position_state_.resize(5, 0);
    velocity_state_.resize(7, 0);
    position_commands_.resize(5, 0);
    velocity_commands_.resize(7, 0);
  }
  else
  {
    velocity_state_.resize(3, 0);
    velocity_commands_.resize(3, 0);
  }

  protocol_version_ = info_.hardware_parameters["protocol_version"];  // should be 1.6
  external_control_port_ = std::stoi(info_.hardware_parameters["port"]);
  if (
    info.hardware_parameters.find("velocity_command_timeout_ms") != info.hardware_parameters.end())
  {
    vel_cmd_timeout_ms_ = std::stoi(info_.hardware_parameters["velocity_command_timeout_ms"]);
  }
  else
  {
    RCLCPP_INFO(
      rclcpp::get_logger("OmnimoveExternalControl"), "velocity timeout not set. Using 500 ms");

    vel_cmd_timeout_ms_ = 500;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("OmnimoveExternalControl"), "port of external control server:%d",
    external_control_port_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KukaUTV3HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  return SystemInterface::on_configure(previous_state);
}

hardware_interface::CallbackReturn KukaUTV3HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  return SystemInterface::on_cleanup(previous_state);
}

hardware_interface::CallbackReturn KukaUTV3HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  client_socket_->shutdown(client_socket_->shutdown_both);
  acceptor_->close();
  return SystemInterface::on_shutdown(previous_state);
}

CallbackReturn KukaUTV3HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  try
  {
    acceptor_.reset(
      new tcp::acceptor(io_context_, tcp::endpoint(tcp::v4(), external_control_port_)));
  }
  catch (std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("OmnimoveExternalControl"), "Failed to start acceptor %s %d", e.what(),
      external_control_port_);
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("OmnimoveExternalControl"), "Waiting for External Control Connection");
  // can i wait for a maximum amount of time
  try
  {
    client_socket_ = std::make_unique<tcp::socket>(io_context_);
  }
  catch (std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("OmnimoveExternalControl"), "Failed to initialise socket %s", e.what());
    return CallbackReturn::ERROR;
  }
  try
  {
    acceptor_->accept(*(client_socket_.get()));
  }
  catch (std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("OmnimoveExternalControl"), "Failed to accept connection %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_ERROR(rclcpp::get_logger("OmnimoveExternalControl"), "Accepted connection");

  return SystemInterface::on_activate(previous_state);
}
hardware_interface::CallbackReturn KukaUTV3HardwareInterface::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  return SystemInterface::on_error(previous_state);
}

std::vector<StateInterface> KukaUTV3HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interface;
  if (agv_type_ == "caterpillar")
  {
    state_interface.emplace_back("move_x", "velocity", &velocity_state_[0]);
    state_interface.emplace_back("move_theta", "velocity", &velocity_state_[1]);
    state_interface.emplace_back("pillar1", "velocity", &velocity_state_[2]);
    state_interface.emplace_back("pillar2", "velocity", &velocity_state_[3]);
    state_interface.emplace_back("pillar3", "velocity", &velocity_state_[4]);
    state_interface.emplace_back("pillar4", "velocity", &velocity_state_[5]);
    state_interface.emplace_back("shield", "velocity", &velocity_state_[6]);
    state_interface.emplace_back("pillar1", "position", &position_state_[0]);
    state_interface.emplace_back("pillar2", "position", &position_state_[1]);
    state_interface.emplace_back("pillar3", "position", &position_state_[2]);
    state_interface.emplace_back("pillar4", "position", &position_state_[3]);
    state_interface.emplace_back("shield", "position", &position_state_[4]);
  }
  else
  {
    state_interface.emplace_back("move_x", "velocity", &velocity_state_[0]);
    state_interface.emplace_back("move_y", "velocity", &velocity_state_[1]);
    state_interface.emplace_back("move_theta", "velocity", &velocity_state_[2]);
  }
  return state_interface;
}

std::vector<CommandInterface> KukaUTV3HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interface;
  if (agv_type_ == "caterpillar")
  {
    command_interface.emplace_back("move_x", "velocity", &velocity_commands_[0]);
    command_interface.emplace_back("move_theta", "velocity", &velocity_commands_[1]);
    command_interface.emplace_back("pillar1", "position", &position_commands_[0]);
    command_interface.emplace_back("pillar2", "position", &position_commands_[1]);
    command_interface.emplace_back("pillar3", "position", &position_commands_[2]);
    command_interface.emplace_back("pillar4", "position", &position_commands_[3]);
    command_interface.emplace_back("shield", "position", &position_commands_[4]);
  }
  else
  {
    command_interface.emplace_back("move_x", "velocity", &velocity_commands_[0]);
    command_interface.emplace_back("move_y", "velocity", &velocity_commands_[1]);
    command_interface.emplace_back("move_theta", "velocity", &velocity_commands_[2]);
  }

  return command_interface;
}

ExternalControlData KukaUTV3HardwareInterface::parseLastMessageFromBuffer()
{
  // assume that sequence will start from the beginning
  const int expected_data_size = ExternalControlData::totalMessageLength();

  size_t last_message_start = (read_buffer_.size() / expected_data_size - 1) * expected_data_size;
  if (last_message_start > read_buffer_.size())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("OmnimoveExternalControl"),
      "Message too small last_message_start %lu, read_buffer_size %lu", last_message_start,
      read_buffer_.size());
    return ExternalControlData();
  }

  for (size_t i = 0; i < last_message_start; ++i)
  {
    read_buffer_.pop_front();
  }
  // now let's check the header.
  std::unique_ptr<char[]> msg_data = std::make_unique<char[]>(read_buffer_.size());
  for (size_t i = 0; i < read_buffer_.size(); ++i)
  {
    msg_data.get()[i] = read_buffer_[i];
  }
  if (!ExternalControlData::isMessageValid(msg_data.get(), read_buffer_.size()))
  {
    const char * EXTERNAL_CONTROL_DATA_HEADER = "KMRUTV03";

    for (size_t i = 0; i < strlen(EXTERNAL_CONTROL_DATA_HEADER); ++i)
    {
      if (read_buffer_[i] != EXTERNAL_CONTROL_DATA_HEADER[i])
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("OmnimoveExternalControl"), "msg_data is %d, header is %d",
          read_buffer_[i], EXTERNAL_CONTROL_DATA_HEADER[i]);
      }
    }

    return ExternalControlData();
  }
  ExternalControlData parsedData(msg_data.get() + 8);
  read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + expected_data_size);
  return parsedData;
}

hardware_interface::return_type KukaUTV3HardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::array<char, 1024> buffer;
  if (client_socket_.get() == NULL)
  {
    RCLCPP_ERROR(rclcpp::get_logger("OmnimoveExternalControl"), "socket is null !!!");
    return return_type::ERROR;
  }
  size_t bytes_received = client_socket_->read_some(boost::asio::buffer(buffer));

  read_buffer_.insert(read_buffer_.end(), buffer.begin(), buffer.begin() + bytes_received);

  // parse all messages from the buffer.
  // incomplete messages need to be kept for further reading.
  ExternalControlData readData = parseLastMessageFromBuffer();

  if (readData.isDataValid())
  {
    if (agv_type_ == "caterpillar")
    {
      velocity_state_[0] = static_cast<double>(readData.speedX());
      velocity_state_[1] = static_cast<double>(readData.speedW());
      position_state_[0] = static_cast<double>(readData.posPillar1());
      velocity_state_[2] = static_cast<double>(readData.speedPillar1());
      position_state_[1] = static_cast<double>(readData.posPillar2());
      velocity_state_[3] = static_cast<double>(readData.speedPillar2());
      position_state_[2] = static_cast<double>(readData.posPillar3());
      velocity_state_[4] = static_cast<double>(readData.speedPillar3());
      position_state_[3] = static_cast<double>(readData.posPillar4());
      velocity_state_[5] = static_cast<double>(readData.speedPillar4());
      position_state_[4] = static_cast<double>(readData.posShield());
      velocity_state_[6] = static_cast<double>(readData.speedShield());
    }
    else
    {
      velocity_state_[0] = static_cast<double>(readData.speedX());
      velocity_state_[1] = static_cast<double>(readData.speedY());
      velocity_state_[2] = static_cast<double>(readData.speedW());
    }
  }
  return return_type::OK;
}

hardware_interface::return_type KukaUTV3HardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (velocity_commands_ == last_sent_velocity_commands_)
  {
    // velocity_commands need to constantly change. This acts as a virtual dead man's switch.
    if (
      (boost::chrono::system_clock::now() - last_sent_velocity_time_) >
      boost::chrono::milliseconds(vel_cmd_timeout_ms_))
    {
      // send a stop command if no new velocity commands are sent.
      client_socket_->send(ExternalControlStopCommand().getSerialisedData());
    }
  }
  else
  {
    last_sent_velocity_commands_ = velocity_commands_;
    last_sent_velocity_time_ = boost::chrono::system_clock::now();

    if (agv_type_ == "caterpillar")
    {
      client_socket_->send(ExternalControlCaterpillarDriveCommand(
                             velocity_commands_[0], velocity_commands_[1], position_commands_[0],
                             position_commands_[1], position_commands_[2], position_commands_[3],
                             position_commands_[4])
                             .getSerialisedData());
    }
    else
    {
      client_socket_->send(ExternalControlOmnimoveDriveCommand(
                             velocity_commands_[0], velocity_commands_[1], velocity_commands_[2])
                             .getSerialisedData());
    }
  }
  return return_type::OK;
}

}  // namespace omnimove
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(omnimove::KukaUTV3HardwareInterface, hardware_interface::SystemInterface)

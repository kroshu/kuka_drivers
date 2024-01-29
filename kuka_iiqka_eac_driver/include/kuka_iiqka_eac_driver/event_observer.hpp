#ifndef KUKA_IIQKA_EAC_DRIVER__EVENT_OBSERVER_HPP_
#define KUKA_IIQKA_EAC_DRIVER__EVENT_OBSERVER_HPP_


#include "rclcpp/macros.hpp"

#include "kuka/external-control-sdk/common/irobot.h"
#include "kuka_iiqka_eac_driver/hardware_interface.hpp"

namespace kuka_eac
{

class KukaEACEventObserver : public kuka::external::control::EventHandler {
public:

    KukaEACEventObserver(KukaEACHardwareInterface* hw_interface) : hw_interface_(hw_interface) {}
    void OnSampling() override 
    {
        RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "External control is active");
    }
    void OnControlModeSwitch(const std::string& reason) override 
    {
        RCLCPP_INFO(
          rclcpp::get_logger("KukaEACHardwareInterface"), "Control mode switch is in progress");
        RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), reason.c_str());
    }
    void OnStopped(const std::string& reason) override 
    {
        RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "External control finished");
        RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), reason.c_str());
        hw_interface_->on_deactivate(hw_interface_->get_state());

    }
    void OnError(const std::string& reason) override 
    {
        RCLCPP_ERROR(
          rclcpp::get_logger("KukaEACHardwareInterface"), "External control stopped by an error");
        RCLCPP_ERROR(rclcpp::get_logger("KukaEACHardwareInterface"), reason.c_str());
        hw_interface_->on_deactivate(hw_interface_->get_state());
    }

private:
    KukaEACHardwareInterface* hw_interface_;
};

}

#endif // KUKA_IIQKA_EAC_DRIVER__EVENT_OBSERVER_HPP_
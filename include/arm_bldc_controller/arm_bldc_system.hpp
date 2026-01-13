#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/macros.hpp>

#include "nmotion_transport/actuator.hpp"
#include "nmotion_transport/interface.hpp"
#include "nmotion_transport/usb_interface.hpp"

#include <rclcpp/time.hpp>
#include <vector>
#include <string>

namespace arm_bldc_controller
{

class ArmBLDCSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmBLDCSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override;

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  Interface *iface_{nullptr};

  std::vector<Actuator*> actuators_;
  std::vector<uint32_t> node_ids_;

  std::vector<double> pos_, vel_, eff_;
  std::vector<double> cmd_pos_, cmd_vel_, cmd_eff_;

  std::string device_{"/dev/ttyACM0"};
};

}  // namespace arm_bldc_controller

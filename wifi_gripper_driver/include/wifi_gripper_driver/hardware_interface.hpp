
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "wifi_gripper_driver/visibility_control.h"

#include "wifi_gripper_driver/wifi_gripper_interface.hpp"

namespace wifi_gripper_driver
{
class WifiGripperHardwareInterface : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WifiGripperHardwareInterface)

  WifiGripperHardwareInterface();
  
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  
  WIFI_GRIPPER_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  WIFI_GRIPPER_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  WIFI_GRIPPER_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  WIFI_GRIPPER_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:

  std::shared_ptr<WifiGripperInterface> gripper_interface_;
  std::thread command_interface_;

  bool command_interface_is_running_;
  double gripper_position_;
  double gripper_velocity_;
  double gripper_position_command_ ;

  std::atomic<float_t> write_command_;
  std::atomic<float_t> gripper_current_state_;

  rclcpp::executors::SingleThreadedExecutor executor;  //Executor needed to subscriber  
};

}  // namespace wifi_gripper_driver

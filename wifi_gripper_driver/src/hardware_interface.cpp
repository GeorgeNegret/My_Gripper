
#include "wifi_gripper_driver/hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


const auto kLogger = rclcpp::get_logger("WifiGripperHardwareInterface");

namespace wifi_gripper_driver
{

WifiGripperHardwareInterface::WifiGripperHardwareInterface()
{
}

hardware_interface::CallbackReturn
WifiGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read parameters from wifi_gripper.ros2_control.xacro file
  // hw_start_sec_ = stod(info_.hardware_parameters["hw_start_duration_sec"]);
  // hw_stop_sec_ = stod(info_.hardware_parameters["hw_stop_duration_sec"]);
  // hw_slowdown_ = stod(info_.hardware_parameters["hw_slowdown"]);
  
  gripper_position_ = std::numeric_limits<double>::quiet_NaN(); 
  gripper_velocity_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_command_ = std::numeric_limits<double>::quiet_NaN();

  const hardware_interface::ComponentInfo& joint = info_.joints[0];

  // There is one command interface: position.
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu command interfaces found. 1 expected.", 
                  joint.name.c_str(), joint.command_interfaces.size());
    return CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %s command interfaces found. '%s' expected.", 
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }

  // There are two state interfaces: position and velocity.
  if (joint.state_interfaces.size() != 2)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                 joint.state_interfaces.size());
    return CallbackReturn::ERROR;
  }

  for (int i = 0; i < 2; ++i)
  {
    if (!(joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(kLogger, "Joint '%s' has %s state interface. Expected %s or %s.", joint.name.c_str(),
                   joint.state_interfaces[i].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  // Create the interface to the gripper.
  gripper_interface_ = std::make_shared<WifiGripperInterface>();
  executor.add_node(gripper_interface_);
  std::thread([this]() { executor.spin(); }).detach();
  //std::thread([&executor]() {executor.spin(); }).detach();
  
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WifiGripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));

  return state_interfaces;
}



std::vector<hardware_interface::CommandInterface> WifiGripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_command_));
  
  return command_interfaces;
}

hardware_interface::CallbackReturn
WifiGripperHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // set some default values for joints
  if (std::isnan(gripper_position_))
  {
    gripper_position_ = 0;
    gripper_velocity_ = 0 ;
    gripper_position_command_ = 0;
  } 
  RCLCPP_INFO(kLogger, "Successfully activated!");
  command_interface_is_running_ = true;
 
  command_interface_ = std::thread([this] {
    // Read from and write to the gripper at 5 Hz.
    auto io_interval = std::chrono::milliseconds(200);
    auto last_io = std::chrono::high_resolution_clock::now();
    while (command_interface_is_running_)
    {
      auto now = std::chrono::high_resolution_clock::now();
      if (now - last_io > io_interval)
      {
        // Write the latest command to the gripper on topic : "gripper/cmd" .  
        this->gripper_interface_->setGripperPosition(write_command_.load());
        
        // Read the state of the gripper from the topic : "gripper/state" .
        gripper_current_state_.store(this->gripper_interface_->getGripperPosition()) ;

        last_io = now;
      }       
    
    }
  });

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
WifiGripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  command_interface_is_running_ = false;
  command_interface_.join();

  return CallbackReturn::SUCCESS;
}
 
hardware_interface::return_type WifiGripperHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // RCLCPP_INFO(kLogger, "gripper_current_state_ : '%f'  ", gripper_current_state_.load());
  gripper_position_ =  gripper_current_state_.load();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WifiGripperHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{      
  //RCLCPP_INFO(kLogger, "gripper_position_command_ : '%f'  ", gripper_position_command_);

  write_command_.store(float_t(gripper_position_command_));

  return hardware_interface::return_type::OK;
}

}  // namespace wifi_gripper_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(wifi_gripper_driver::WifiGripperHardwareInterface, hardware_interface::ActuatorInterface)

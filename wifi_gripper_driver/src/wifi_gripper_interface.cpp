
#include "wifi_gripper_driver/wifi_gripper_interface.hpp"

#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


using std::placeholders::_1;


WifiGripperInterface::WifiGripperInterface() : Node("wifi_gripper_interface")
  {
    // auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    auto best_effort_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    gripper_sub_ = this->create_subscription<std_msgs::msg::Float32>("gripper/state", best_effort_qos, 
                           std::bind(&WifiGripperInterface::getLastPosition, this, _1)
                        );
              
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float32>("gripper/cmd", 10);
  }


void WifiGripperInterface::getLastPosition(const std_msgs::msg::Float32::SharedPtr msg)
  {
       lastPosition_ = msg->data;
   
  }
void WifiGripperInterface::setGripperPosition(const std::float_t& cmd)
  {
    auto command = std_msgs::msg::Float32();
    command.data = cmd ;
    //RCLCPP_INFO(this->get_logger(), "RUN setGripperPosition ........................ !");
    gripper_pub_->publish(command);
  }

double_t WifiGripperInterface::getGripperPosition() const
 {
    //RCLCPP_INFO(this->get_logger(), "lastPosition_ : '%f'", lastPosition_);
    return lastPosition_ ;
 }  




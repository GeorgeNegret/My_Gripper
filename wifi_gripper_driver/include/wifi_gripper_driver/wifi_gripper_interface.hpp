
#pragma once
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

/**
 * @brief This class is responsible for communicating with the gripper via a WiFi, and maintaining a record of
 * the gripper's current state.
 *
 */
class WifiGripperInterface : public rclcpp::Node
{
public:
  WifiGripperInterface();

   /**
   * @brief Publish on topic "gripper/cmd"  commands the gripper to move to the desired position.
   */
  void setGripperPosition(const std::float_t& cmd);
  
    /**
   * @brief Read on topic "gripper/state" the current position of the gripper.
   */
  void getLastPosition(const std_msgs::msg::Float32::SharedPtr msg) ;
  
  double_t getGripperPosition() const;
  

 private:

  float_t lastPosition_ ;


  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_sub_;
   
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;

};

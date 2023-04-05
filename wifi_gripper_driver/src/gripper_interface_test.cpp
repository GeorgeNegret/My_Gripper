#include <iostream>
#include <thread>
#include <chrono>
#include "moveit/move_group_interface/move_group_interface.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gripper_control");

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_gripper_node);
    std::thread([&executor]() {executor.spin();}).detach();


    MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
    // move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
    // move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
    auto gripper_joint_values = move_group_gripper.getCurrentJointValues();


    std::cout << "Closing gripper...\n";
    gripper_joint_values[0] = 0.035;
    move_group_gripper.setJointValueTarget(gripper_joint_values);
    move_group_gripper.move();
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    std::cout << "Opening gripper...\n";
    gripper_joint_values[0] = 0.0;
    move_group_gripper.setJointValueTarget(gripper_joint_values);
    move_group_gripper.move();
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    std::cout << "Closing gripper...\n";
    gripper_joint_values[0] = 0.035;
    move_group_gripper.setJointValueTarget(gripper_joint_values);
    move_group_gripper.move();
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));
    

    std::cout << "Opening gripper...\n";
    gripper_joint_values[0] = 0.0;
    move_group_gripper.setJointValueTarget(gripper_joint_values);
    move_group_gripper.move();
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    
    // shutdown
    rclcpp::shutdown();

    return 0;
}



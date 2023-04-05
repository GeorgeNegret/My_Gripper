#include <iostream>
#include <thread>

#include <wifi_gripper_driver/hardware_interface.hpp>


int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    
    auto gripper = std::make_unique<wifi_gripper_driver::WifiGripperHardwareInterface>();

    std::cout << "Closing gripper...\n";
    gripper->test_gripper(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));


    std::cout << "Opening gripper...\n";
    gripper->test_gripper(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    std::cout << "Closing gripper...\n";
    gripper->test_gripper(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));


    std::cout << "Opening gripper...\n";
    gripper->test_gripper(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));


    // shutdown
    rclcpp::shutdown();

    return 0;
}
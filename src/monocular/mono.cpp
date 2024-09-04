#include <iostream>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"
#include "System.h"

bool should_shutdown = false;

void signal_handler(int signum)
{
    (void)signum;  // Cast to void to avoid unused parameter warning
    should_shutdown = true;
}

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    // Set up signal handler
    signal(SIGINT, signal_handler);

    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);

    try
    {
        node->Run();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
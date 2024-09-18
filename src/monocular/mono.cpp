#include <iostream>
<<<<<<< HEAD
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
=======
   #include <algorithm>
   #include <fstream>
   #include <chrono>
   #include <thread>

   #include "rclcpp/rclcpp.hpp"
   #include "monocular-slam-node.hpp"

   #include "System.h"
   #include "ORBVocabulary.h"

   int main(int argc, char **argv)
   {
       if(argc < 3)
       {
           std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
           return 1;
       }
>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24

       rclcpp::init(argc, argv);

<<<<<<< HEAD
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
=======
       // Load ORB Vocabulary
       std::cout << "Loading ORB Vocabulary. This could take a while..." << std::endl;
       ORB_SLAM3::ORBVocabulary* pVocabulary = new ORB_SLAM3::ORBVocabulary();
       bool bVocLoad = false;
       try
       {
           bVocLoad = pVocabulary->loadFromTextFile(argv[1]);
           if(!bVocLoad)
           {
               std::cerr << "Wrong path to vocabulary. " << std::endl;
               return 1;
           }
       }
       catch(std::exception& e)
       {
           std::cerr << e.what() << std::endl;
           return 1;
       }

       // Create SLAM system. It initializes all system threads and gets ready to process frames.
       bool visualization = true;
       ORB_SLAM3::System SLAM(pVocabulary, argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

       auto node = std::make_shared<MonocularSlamNode>(&SLAM);
       std::cout << "============================ " << std::endl;

       rclcpp::spin(node);
       rclcpp::shutdown();

       delete pVocabulary;

       return 0;
   }
>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24

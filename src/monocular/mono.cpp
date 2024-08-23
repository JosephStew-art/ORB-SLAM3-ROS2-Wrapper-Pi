#include <iostream>
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

       rclcpp::init(argc, argv);

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

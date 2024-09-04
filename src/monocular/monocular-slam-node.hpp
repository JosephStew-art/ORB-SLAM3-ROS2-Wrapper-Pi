#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "System.h"
#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();

    void Run();  // Make Run() public

private:
    ORB_SLAM3::System* m_SLAM;
    cv::VideoCapture m_cap;
    bool b_continue_session;
};

#endif
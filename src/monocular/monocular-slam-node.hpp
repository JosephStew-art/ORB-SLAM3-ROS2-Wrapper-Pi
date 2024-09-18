#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
<<<<<<< HEAD
#include <opencv2/opencv.hpp>
=======
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24
#include "System.h"
#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();

    void Run();  // Make Run() public

private:
<<<<<<< HEAD
    ORB_SLAM3::System* m_SLAM;
    cv::VideoCapture m_cap;
    bool b_continue_session;
};

#endif
=======
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;
    cv_bridge::CvImageConstPtr m_cvImPtr;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
};

#endif // __MONOCULAR_SLAM_NODE_HPP__
>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24

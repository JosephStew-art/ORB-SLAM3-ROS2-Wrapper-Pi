#include "monocular-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
    : Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "MonocularSlamNode initialized");
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SavePointCloud("pointcloud.txt");
}

void MonocularSlamNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvShare(msg, "mono8");
        RCLCPP_INFO(this->get_logger(), "Received image at time: %d.%d", 
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = m_cvImPtr->image.clone();

    // Check if the image is empty
    if(image.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Received empty image");
        return;
    }

    // Check if the image is already grayscale
    if(image.channels() != 1)
    {
        RCLCPP_WARN(this->get_logger(), "Received non-grayscale image. Converting to grayscale.");
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }

    // Check image dimensions
    if(image.cols != 640 || image.rows != 480)
    {
        RCLCPP_WARN(this->get_logger(), "Received image with unexpected dimensions: %dx%d. Resizing to 640x300.", image.cols, image.rows);
        cv::resize(image, image, cv::Size(640, 480));
    }

    RCLCPP_INFO(this->get_logger(), "Processing frame");
    m_SLAM->TrackMonocular(image, Utility::StampToSec(msg->header.stamp));
}
#include "monocular-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
    : Node("ORB_SLAM3_ROS2"), m_first_frame(true)
{
    m_SLAM = pSLAM;
    m_image_subscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "camera/image_raw/compressed",
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

void MonocularSlamNode::GrabImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try
    {
        // Decompress image
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        // Check if the image is empty
        if(image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Received empty image");
            return;
        }

        // Convert to grayscale
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

        // Check image dimensions
        if(gray_image.cols != 640 || gray_image.rows != 480)
        {
            RCLCPP_WARN(this->get_logger(), "Received image with unexpected dimensions: %dx%d. Resizing to 640x480.", gray_image.cols, gray_image.rows);
            cv::resize(gray_image, gray_image, cv::Size(640, 480));
        }

        // Normalize timestamp
        double timestamp;
        if (m_first_frame)
        {
            m_initial_timestamp = msg->header.stamp;
            timestamp = 0.0;
            m_first_frame = false;
        }
        else
        {
            rclcpp::Duration diff = rclcpp::Time(msg->header.stamp) - m_initial_timestamp;
            timestamp = diff.seconds();
        }

        m_SLAM->TrackMonocular(gray_image, timestamp);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

void MonocularSlamNode::Run()
{
    rclcpp::spin(shared_from_this());
}
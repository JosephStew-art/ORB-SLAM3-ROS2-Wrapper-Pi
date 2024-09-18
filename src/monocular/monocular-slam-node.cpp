#include "monocular-slam-node.hpp"
<<<<<<< HEAD
#include <chrono>

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
    : Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM), b_continue_session(true)
{
    // Initialize webcam
    m_cap.open(0);
    if (!m_cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Error: Couldn't open the camera.");
        throw std::runtime_error("Failed to open camera");
    }

    // Set camera parameters (you may need to adjust these)
    m_cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    m_cap.set(cv::CAP_PROP_FPS, 30);

=======
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
>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24
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

<<<<<<< HEAD
void MonocularSlamNode::Run()
{
    cv::Mat frame;
    auto start_time = std::chrono::steady_clock::now();

    while (b_continue_session && rclcpp::ok())
    {
        // Capture frame
        m_cap >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Blank frame grabbed");
            break;
        }

        // Get timestamp
        auto now = std::chrono::steady_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() / 1000.0;

        // Pass the image to the SLAM system
        m_SLAM->TrackMonocular(frame, timestamp);

        // Process any pending callbacks
        rclcpp::spin_some(this->get_node_base_interface());
    }
=======
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
>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24
}
#include "monocular-slam-node.hpp"
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
}
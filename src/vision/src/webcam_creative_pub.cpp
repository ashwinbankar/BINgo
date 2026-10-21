#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// Changed class name to match your custom constructor
class WebcamCreativePublisher : public rclcpp::Node
{
public:
    WebcamCreativePublisher() : Node("webcam_creative_pub")
    {
        // Changed topic to be specific to the Creative camera
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/creative/image_raw", 10);
        
        // Open the default camera (index 0); webcam might be index 2 or 4
        cap_.open(2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
            return;
        }

        // Set a timer to capture and publish frames at ~30Hz (33ms)
        timer_ = this->create_wall_timer(
                 33ms, std::bind(&WebcamCreativePublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Creative webcam publisher node initialized.");
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame; // Capture a new frame
    
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured!");
            return;
        }

        // Swap Red and Blue channels to fix the blue tint
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

        // Convert OpenCV Mat to ROS2 Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // Stamp the message with current time
        msg->header.stamp = this->now();

        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Updated to spin the newly named class
    rclcpp::spin(std::make_shared<WebcamCreativePublisher>());
    rclcpp::shutdown();
    return 0;
}
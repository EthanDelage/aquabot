#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
    timer_ = this->create_wall_timer(
        2s, std::bind(&MinimalImagePublisher::timer_callback, this));
  }
 
private:
  void timer_callback() {
    // Create a new 640x480 image
    cv::Mat my_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);
 
    // Generate an image where each pixel is a random color
    cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    // Declare the text position
    cv::Point text_position((40 * count_) % IMAGE_WIDTH, 50);
    
    // Declare the size and color of the font
    int font_size = 2;
    cv::Scalar font_color(255, 55, 55);
    
    // Declare the font weight
    int font_weight = 4;
    
    // Put the text in the image
    cv::putText(my_image, "Aquabot", text_position, cv::FONT_HERSHEY_SIMPLEX, font_size, font_color, font_weight);
    
    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image)
               .toImageMsg();

    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
};


int main(int argc, char ** argv) 
{
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

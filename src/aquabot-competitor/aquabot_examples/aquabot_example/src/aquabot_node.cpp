#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

#define MAX_THRUSTER_POS M_PI/2

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class AquabotNode : public rclcpp::Node
{
  public:
    AquabotNode() : Node("aquabot_node_cpp"), 
    m_target_pos(1.0), 
    m_current_pos(0.0)
    {
      // Log that the node has succesfully started
      RCLCPP_INFO(this->get_logger(), "Hello world from aquabot_node_cpp!");

      // Create a publisher on the topic "status_string" that will publish a std_msgs::msg::String message
      m_thrustersPos_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/main/pos", 10);

      // Create a timer that will call the timer_callback function every 500ms
      m_tf2_timer = this->create_wall_timer(100ms, std::bind(&AquabotNode::update_engine_position, this));
      m_timer = this->create_wall_timer(10s, std::bind(&AquabotNode::timer_callback, this));

      // Create tf2 buffer and listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
    void timer_callback()
    {
      m_target_pos = -m_target_pos;
      auto message = std_msgs::msg::Float64();
      message.data = m_target_pos;
      m_thrustersPos_pub->publish(message);
      RCLCPP_INFO(this->get_logger(), "New target pos: '%f'", m_target_pos);
    }

    void update_engine_position()
    {
      // Get the transform (relative engine position from wamv)
      geometry_msgs::msg::TransformStamped transformStamped;
      try
      {
        transformStamped = tf_buffer_->lookupTransform("wamv", "wamv/wamv/main_engine_link", tf2::TimePointZero);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
      }

      // Convert to RPY
      tf2::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      // Update current position
      m_current_pos = yaw;
      RCLCPP_INFO(this->get_logger(), "Current pos: '%f'", m_current_pos);
    }

    // Declare variables
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersPos_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::TimerBase::SharedPtr m_tf2_timer;

    float m_target_pos;
    float m_current_pos;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AquabotNode>());
  rclcpp::shutdown();
  return 0;
}

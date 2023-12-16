#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class ExampleNode : public rclcpp::Node
{
  public:
    ExampleNode() : Node("example_node_cpp"), m_loop_count(0)
    {
      // Log that the node has succesfully started
      RCLCPP_INFO(this->get_logger(), "Hello world from example_node_cpp!");

      // Create a publisher on the topic "status_string" that will publish a std_msgs::msg::String message
      m_status_pub = this->create_publisher<std_msgs::msg::String>("status_string", 10);

      // Create a timer that will call the timer_callback function every 500ms
      m_timer = this->create_wall_timer(500ms, std::bind(&ExampleNode::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Node name : " + std::string(this->get_name ()) 
                    + ", Loop : " + std::to_string(m_loop_count++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      m_status_pub->publish(message); // Publish our std_msgs::msg::String message (topic "status_string")
    }

    // Declare variables
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_status_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    size_t m_loop_count;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleNode>());
  rclcpp::shutdown();
  return 0;
}

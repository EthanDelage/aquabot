#include <rclcpp/rclcpp.hpp>
#include "Pathfinding.hpp"

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Pathfinding>());
	rclcpp::shutdown();
	return (0);
}
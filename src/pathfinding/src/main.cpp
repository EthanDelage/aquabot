#include "Pathfinding.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Pathfinding>());
	rclcpp::shutdown();
	return (0);
}
#include "Navigation.hpp"

int	main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Navigation>());
	rclcpp::shutdown();
	return 0;
}
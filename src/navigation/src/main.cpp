#include "Navigation.hpp"

int	main(int argc, char** argv) {
	double	gain;
	double	sigma;

	rclcpp::init(argc, argv);
	if (argc == 3) {
		gain = strtod(argv[1], NULL);
		sigma = strtod(argv[2], NULL);
		rclcpp::spin(std::make_shared<Navigation>(gain, sigma));
	} else {
		rclcpp::spin(std::make_shared<Navigation>());
	}
	rclcpp::shutdown();
	return 0;
}

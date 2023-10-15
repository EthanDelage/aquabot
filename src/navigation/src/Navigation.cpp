#include "Navigation.hpp"

#include <algorithm>

using std::placeholders::_1;

Navigation::Navigation() : Node("navigation") {
	_pinger = create_subscription<ros_gz_interfaces::msg::ParamVec>("/wamv/sensors/acoustics/receiver/range_bearing", 10,
				std::bind(&Navigation::pingerCallback, this, _1));
	_publisherThrust = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/main/thrust", 5);
	_publisherPos = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/main/pos", 5);
}

void Navigation::pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg) {
	for (const auto &param: msg->params) {
		std::string name = param.name;

		if (name == "bearing")
			_bearing = param.value.double_value;
		else if (name == "range")
			_range = param.value.double_value;
	}
	setHeading(_bearing, _range);
}

void Navigation::setHeading(double bearing, double range) {
	auto 	posMsg = std_msgs::msg::Float64();
	auto 	thrustMsg = std_msgs::msg::Float64();
	double	regulation;

	regulation = regulator(bearing);
	posMsg.data = (POS_MIN + regulation * 2 * POS_MAX);
	thrustMsg.data = std::abs(std::abs(regulation - 0.5) - 0.5) * 2 * THRUST_MAX;
	if (range < 20) {
		posMsg.data = 0;
		thrustMsg.data = 0;
	}
	std::cout << "Bearing: " << bearing << std::endl;
	std::cout << "Range: " << range << std::endl;
	std::cout << "Regulator: " << regulation << std::endl;
	std::cout << "Pos: " << posMsg.data << std::endl;
	std::cout << "Thrust: " << thrustMsg.data << std::endl << std::endl;
	_publisherPos->publish(posMsg);
	_publisherThrust->publish(thrustMsg);
}

double Navigation::regulator(double bearing) {
	const double	goal = 0;
	const double	k = 0.5;
	double			gap;
	double			regulation;

	gap = (goal - bearing) * k;
	regulation = (gap + M_PI) / (2 * M_PI);
	std::cout << "Regulation: " << regulation << std::endl;
	regulation = std::min(regulation, 1.);
	regulation = std::max(regulation, 0.);
	std::cout << "%: " << regulation << std::endl;
	return (regulation);
}
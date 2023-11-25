#include "Navigation.hpp"

#include <algorithm>

using std::placeholders::_1;

Navigation::Navigation() : Node("navigation") {
	std::vector<point_t>	path;
	_gain = 0.2;
	_sigma = 0.05;
	_benchmark = false;
	_pinger = create_subscription<ros_gz_interfaces::msg::ParamVec>("/range_bearing", 10,
				std::bind(&Navigation::pingerCallback, this, _1));
	_alliesPos = create_subscription<sensor_msgs::msg::NavSatFix>("/wamv/ais_sensor/allies_position", 10,
				std::bind(&Navigation::alliesPosCallback, this, _1));
	_publisherThrust = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/main/thrust", 5);
	_publisherPos = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/main/pos", 5);
}

Navigation::Navigation(double gain, double sigma) : Navigation() {
	_gain = gain;
	_sigma = sigma;
	_benchmark = true;
}

void Navigation::pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg) {
	for (const auto &param: msg->params) {
		std::string name = param.name;

		if (name == "bearing") {
			_bearing = param.value.double_value;
			std::cout << "Bearing: " << _bearing << std::endl;
		} else if (name == "range") {
			_range = param.value.double_value;
			std::cout << "Range: " << _range << std::endl;
		} else if (name == "desiredRange") {
			_desiredRange = param.value.double_value;
			std::cout << "Desired range: " << _desiredRange << std::endl;
		}
	}
	setHeading(_bearing, _range);
}

void Navigation::alliesPosCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	double latitude = msg->latitude;
	double longitude = msg->longitude;

	std::cout << "ally latitude: " << latitude << std::endl;
	std::cout << "ally longitude: " << longitude << std::endl;
}

void Navigation::setHeading(double bearing, double range) {
	auto 	posMsg = std_msgs::msg::Float64();
	auto 	thrustMsg = std_msgs::msg::Float64();
	double	regulation;

	regulation = regulator(bearing);
	posMsg.data = (POS_MIN + regulation * 2 * POS_MAX);
	thrustMsg.data = std::abs(std::abs(regulation - 0.5) - 0.5) * 2 * THRUST_MAX;
	thrustMsg.data = calculateThrust(regulation);
	if (range < _desiredRange) {
		posMsg.data = 0;
		thrustMsg.data = 0;
	}
	_publisherPos->publish(posMsg);
	_publisherThrust->publish(thrustMsg);
	if (range < MAX_BUOY_RANGE && _benchmark)
		rclcpp::shutdown();
}

double Navigation::calculateThrust(double regulation) {
	const double amplitude = THRUST_MAX - NAV_THRUST_MIN;
	const double mean = 0.5;
	double thrust;

	thrust = amplitude * std::exp(-std::pow(((regulation - mean)) / (_sigma * 2), 2));
	thrust += NAV_THRUST_MIN;
	return (thrust);
}

double Navigation::regulator(double bearing) {
	const double	goal = 0;
	double			gap;
	double			regulation;

	gap = (goal - bearing) * _gain;
	regulation = (gap + M_PI) / (2 * M_PI);
	regulation = std::min(regulation, 1.);
	regulation = std::max(regulation, 0.);
	return (regulation);
}
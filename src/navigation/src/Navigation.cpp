#include "Navigation.hpp"
#include "Pathfinding.hpp"

#include <algorithm>

using std::placeholders::_1;

Navigation::Navigation() : Node("navigation") {
	_gain = 0.2;
	_sigma = 0.05;
	_benchmark = false;
	_buoyPing = false;
	try {
		Pathfinding	pathfinding;
	} catch (std::runtime_error const & e) {
		std::cerr << e.what() << std::endl;
		exit(1);
	}
	_pinger = create_subscription<ros_gz_interfaces::msg::ParamVec>("/wamv/sensors/acoustics/receiver/range_bearing", 10,
				std::bind(&Navigation::pingerCallback, this, _1));
	_gps = create_subscription<sensor_msgs::msg::NavSatFix>("/wamv/sensors/gps/gps/fix", 10,
				std::bind(&Navigation::gpsCallback, this, _1));
	_imu = create_subscription<sensor_msgs::msg::Imu>("/wamv/sensors/imu/imu/data", 10,
				std::bind(&Navigation::imuCallback, this, _1));
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

		if (name == "bearing")
			_bearing = param.value.double_value;
		else if (name == "range")
			_range = param.value.double_value;
	}
	_buoyPing = true;
	setHeading(_bearing, _range);
}

void Navigation::gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	double latitude = msg->latitude;
	double longitude = msg->longitude;

	calculateMapPos(latitude, longitude);
}

void Navigation::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
	_orientation = msg->orientation;
	double	yaw;

	// roll (x-axis rotation)
//	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
//	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
//	angles.roll = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
//	double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
//	double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
//	angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (_orientation.w * _orientation.z + _orientation.x * _orientation.y);
	double cosy_cosp = 1 - 2 * (_orientation.y * _orientation.y + _orientation.z * _orientation.z);
	yaw = std::atan2(siny_cosp, cosy_cosp);
	if (_buoyPing && _gpsPing) {
		_buoyPing = false;
		_gpsPing = false;
		double	buoyOrientation = convertToMinusPiPi(yaw + _bearing);
		_buoyPos.x = _range * std::cos(buoyOrientation) + _boatPos.y;
		_buoyPos.y = _range * std::sin(buoyOrientation) + _boatPos.x;
		std::cout << "Buoy: [" << _buoyPos.x << ", " << _buoyPos.y << "]" << std::endl;
		std::cout << "Boat: [" << _boatPos.x << ", " << _boatPos.y << "]" << std::endl;
	}
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
//	std::cout << "range: " << range << std::endl;
	if (range < MAX_BUOY_RANGE) {
		posMsg.data = 0;
		thrustMsg.data = 0;
	}
	std::cout << "Bearing: " << bearing << std::endl;
//	std::cout << "Range: " << range << std::endl;
//	std::cout << "Regulator: " << regulation << std::endl;
//	std::cout << "Pos: " << posMsg.data << std::endl;
//	std::cout << "Thrust: " << thrustMsg.data << std::endl << std::endl;

//	_publisherPos->publish(posMsg);
//	_publisherThrust->publish(thrustMsg);
	if (range < MAX_BUOY_RANGE && _benchmark)
		rclcpp::shutdown();
}

void	Navigation::calculateMapPos(double latitude, double longitude) {
	double	x, y;

	x = (latitude - LATITUDE_0) / (LATITUDE_1 - LATITUDE_0);
	_boatPos.x = x * 600 - 300;
	y = (longitude - LONGITUDE_0) / (LONGITUDE_1 - LONGITUDE_0);
	_boatPos.y = y * 600 - 300;
	_gpsPing = true;
//	std::cout << "Boat: [" << _boatPos.x << ", " << _boatPos.y << "]" << std::endl;
}

double Navigation::calculateThrust(double regulation) {
	const double amplitude = THRUST_MAX - NAV_THRUST_MIN;
	// Position où l'amplitude est atteinte
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
//	std::cout << "Regulation: " << regulation << std::endl;
	regulation = std::min(regulation, 1.);
	regulation = std::max(regulation, 0.);
//	std::cout << "%: " << regulation << std::endl;
	return (regulation);
}

double Navigation::convertToMinusPiPi(double angleRadians) {
	double result = fmod(angleRadians, 2.0 * M_PI);
	if (result > M_PI) {
		result -= 2.0 * M_PI;
	}
	return result;
}
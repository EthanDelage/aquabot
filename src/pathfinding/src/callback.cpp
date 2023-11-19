#include <cmath>

#include "Pathfinding.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"

void Pathfinding::pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg) {
	for (const auto &param: msg->params) {
		std::string name = param.name;

		if (name == "bearing")
			_buoyBearing = param.value.double_value;
		else if (name == "range")
			_buoyRange = param.value.double_value;
	}
	_buoyPing = true;
}

void Pathfinding::gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	double latitude = msg->latitude;
	double longitude = msg->longitude;

	calculateMapPos(latitude, longitude);
}

void Pathfinding::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
	static bool						buoyPosCalculate = false;
	geometry_msgs::msg::Quaternion	orientation;

	orientation = msg->orientation;
	double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
	double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
	_orientation = std::atan2(siny_cosp, cosy_cosp);
	if (_buoyPing && _gpsPing && !buoyPosCalculate) {
		_buoyPing = false;
		_gpsPing = false;
		double	buoyOrientation = convertToMinusPiPi(_orientation + _buoyBearing);
		_buoyPos.x = _buoyRange * std::cos(buoyOrientation) + _boatPos.y;
		_buoyPos.y = _buoyRange * std::sin(buoyOrientation) + _boatPos.x;
		buoyPosCalculate = true;
		addBuoy(_buoyPos);
		std::cout << "Buoy pos: [" << _buoyPos.x << ", " << _buoyPos.y << "]" << std::endl;
	}
}


double Pathfinding::convertToMinusPiPi(double angleRadians) {
	double result = fmod(angleRadians, 2.0 * M_PI);
	if (result > M_PI) {
		result -= 2.0 * M_PI;
	}
	return result;
}

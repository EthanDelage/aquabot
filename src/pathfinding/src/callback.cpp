#include <cmath>

#include "Pathfinding.hpp"

#include "geometry_msgs/msg/quaternion.hpp"

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
	_gpsPing = true;
	if (_buoyPosCalculate && !_pathCalculated) {
		_path = calculatePath(_boatPos);
		std::cout << "boat pos: " << _boatPos.x << ", " << _boatPos.y << std::endl;
		for (auto node : _path)
			std::cout << "[" << node.x << "," << node.y << "], ";
		std::cout << std::endl;
		_pathCalculated = true;
	}
}

void Pathfinding::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
	calculateYaw(msg->orientation);
	_imuPing = true;
	if (_buoyPing && _gpsPing && !_buoyPosCalculate) {
		calculateBuoyPos();
		_buoyPosCalculate = true;
		addBuoy(_buoyPos);
		std::cout << "Buoy pos: [" << _buoyPos.x << ", " << _buoyPos.y << "]" << std::endl;
	}
	if (_gpsPing && _pathCalculated && !_path.empty()) {
		if (_path[0].x == _buoyPos.x && _path[0].y == _buoyPos.y)
			publishRangeBearing(std::pair<double, double>(_buoyRange, _buoyBearing), 15);
		else
			publishRangeBearing(calculateRangeBearing(), 10);
	}
}

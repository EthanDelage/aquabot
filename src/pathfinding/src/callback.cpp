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

	_boatPos = calculateMapPos(latitude, longitude);
	_gpsPing = true;
	if (_buoyPosCalculate && !_pathCalculated) {
		_path = calculatePath(_boatPos);
		for (auto node : _path)
			std::cout << "[" << node.x << "," << node.y << "], ";
		std::cout << std::endl;
		_pathCalculated = true;
	}
}

void Pathfinding::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
	_orientation = calculateYaw(msg->orientation);
	_imuPing = true;
	if (_buoyPing && _gpsPing && !_buoyPosCalculate)
		addBuoy();
	if (_gpsPing && _pathCalculated && !_path.empty()) {
		if (_path[0].x == _buoy.position.x && _path[0].y == _buoy.position.y)
			publishRangeBearing(std::pair<double, double>(_buoyRange, _buoyBearing), MAX_BUOY_RANGE);
		else
			publishRangeBearing(calculateRangeBearing(), MAX_CHECKPOINT_RANGE);
	}
}

void Pathfinding::alliesCallback(geometry_msgs::msg::PoseArray::SharedPtr msg) {
	std::vector<std::pair<point_t, double>>	closeAllies;

	for (const auto &pose : msg->poses)
	{
		std::pair<point_t, double>	allyInfo;
		allyInfo.first.x = pose.position.x;
		allyInfo.first.y = pose.position.y;
		allyInfo.first = calculateMapPos(allyInfo.first.x, allyInfo.first.y);
		allyInfo.second = calculateYaw(pose.orientation);
		if (calculateDist(_boatPos, allyInfo.first) > MIN_ALLY_RANGE)
			continue;
		closeAllies.push_back(allyInfo);
	}
	if (!closeAllies.empty()) {
		_path = calculatePathWithAllies(_boatPos, closeAllies);
	}
}
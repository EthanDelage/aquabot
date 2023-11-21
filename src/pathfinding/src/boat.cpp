#include "Pathfinding.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include <cmath>

std::vector<point_t> Pathfinding::calculatePath(point_t boatPos) {
	Graph								graph(_obstaclesGraph);
	size_t								boatIndex;
	std::vector<std::pair<size_t, double>>	reversePath;
	std::pair<size_t, double>				current;
	std::list<size_t>						path;

	boatIndex = addBoat(boatPos, graph);

	reversePath = djikstra(boatIndex, _buoyGraphIndex, graph);

	current = reversePath[_buoyGraphIndex];

	path.push_front(_buoyGraphIndex);
	while (current.first != boatIndex) {
		path.push_front(current.first);
		current = reversePath[current.first];
	}
	return (convertNodeToPoint(path));
}

void	Pathfinding::calculateMapPos(double latitude, double longitude) {
	double	x, y;

	x = (longitude - LONGITUDE_0) / (LONGITUDE_1 - LONGITUDE_0);
	_boatPos.x = x * 600 - 300;
	y = (latitude - LATITUDE_0) / (LATITUDE_1 - LATITUDE_0);
	_boatPos.y = y * 600 - 300;
//	std::cout << "Boat: [" << _boatPos.x << ", " << _boatPos.y << "]" << std::endl;
}

void Pathfinding::calculateYaw(const geometry_msgs::msg::Quaternion& orientation) {
	double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
	double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
	_orientation = std::atan2(siny_cosp, cosy_cosp);
}

std::pair<double, double> Pathfinding::calculateRangeBearing() {
	std::pair<double, double>	rangeBearing;
	point_t						checkpointVec;
	point_t						boatVec;

	rangeBearing.first = calculateDist(_boatPos, _path[0]);
	checkpointVec.x = _path[0].x - _boatPos.x;
	checkpointVec.y = _path[0].y - _boatPos.y;
	boatVec.x = std::cos(_orientation);
	boatVec.y = std::sin(_orientation);

	double dotProduct = boatVec.x * checkpointVec.x + boatVec.y * checkpointVec.y;

	double crossProduct = boatVec.x * checkpointVec.y - boatVec.y * checkpointVec.x;

	rangeBearing.second = std::atan2(crossProduct, dotProduct);
//	rangeBearing.second = std::acos(dotProduct / (norm1 * norm2));
//	rangeBearing.second = -convertToMinusPiPi(std::atan2(checkpointVec.y, checkpointVec.x) - _orientation);
	std::cout << "chec orientation: " << rangeBearing.second << std::endl;
	return (rangeBearing);
}

void Pathfinding::publishRangeBearing(const std::pair<double, double>& rangeBearing) {
	auto	paramVecMsg = ros_gz_interfaces::msg::ParamVec();
	rcl_interfaces::msg::Parameter	range;
	rcl_interfaces::msg::Parameter	bearing;

	range.name = "range";
	range.value.double_value = rangeBearing.first;
	bearing.name = "bearing";
	bearing.value.double_value = rangeBearing.second;

	paramVecMsg.params.push_back(range);
	paramVecMsg.params.push_back(bearing);
	if (rangeBearing.first < MIN_CHECKPOINT_RANGE)
		_path.erase(_path.begin());
	_publisherRangeBearing->publish(paramVecMsg);
}
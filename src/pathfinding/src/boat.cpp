#include "Pathfinding.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include <cmath>

size_t Pathfinding::addBoat(point_t boatPos, Graph& graph) {
	size_t	boatIndex;

	boatIndex = graph.addVertex();
	generateNodeAdjList(boatPos, boatIndex, graph);
	if (!isHitObstacle(boatPos, _buoyPos))
		graph.addEdge(boatIndex, _buoyGraphIndex, calculateDist(boatPos, _buoyPos));
	std::cout << std::endl;
	graph.printGraph();
	return (boatIndex);
}

void	Pathfinding::calculateMapPos(double latitude, double longitude) {
	double	x, y;

	x = (longitude - LONGITUDE_0) / (LONGITUDE_1 - LONGITUDE_0);
	_boatPos.x = x * 600 - 300;
	y = (latitude - LATITUDE_0) / (LATITUDE_1 - LATITUDE_0);
	_boatPos.y = y * 600 - 300;
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
	return (rangeBearing);
}

void Pathfinding::publishRangeBearing(const std::pair<double, double>& rangeBearing, double desiredRange) {
	auto	paramVecMsg = ros_gz_interfaces::msg::ParamVec();
	rcl_interfaces::msg::Parameter	rangeMsg;
	rcl_interfaces::msg::Parameter	bearingMsg;
	rcl_interfaces::msg::Parameter	desiredRangeMsg;

	rangeMsg.name = "range";
	rangeMsg.value.double_value = rangeBearing.first;
	bearingMsg.name = "bearing";
	bearingMsg.value.double_value = rangeBearing.second;
	desiredRangeMsg.name = "desiredRange";
	desiredRangeMsg.value.double_value = desiredRange;
	desiredRangeMsg.name = "state";
	desiredRangeMsg.value.integer_value = _state;

	paramVecMsg.params.push_back(rangeMsg);
	paramVecMsg.params.push_back(bearingMsg);
	paramVecMsg.params.push_back(desiredRangeMsg);
	if (rangeBearing.first < desiredRange)
		_path.erase(_path.begin());
	_publisherRangeBearing->publish(paramVecMsg);
}
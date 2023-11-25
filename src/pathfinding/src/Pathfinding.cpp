#include <iterator>

#include "Pathfinding.hpp"

using std::placeholders::_1;

Pathfinding::Pathfinding() :
	Node("pathfinding") {

	_buoyPosCalculate = false;
	_pathCalculated = false;
	_buoyPing = false;
	_gpsPing = false;
	_imuPing = false;

	if (init() == -1) {
		std::cout << "Cannot open " <<  OBSTACLE_FILE << std::endl;
		exit(1);
	}
	_pinger = create_subscription<ros_gz_interfaces::msg::ParamVec>("/wamv/sensors/acoustics/receiver/range_bearing", 10,
				std::bind(&Pathfinding::pingerCallback, this, _1));
	_gps = create_subscription<sensor_msgs::msg::NavSatFix>("/wamv/sensors/gps/gps/fix", 10,
				std::bind(&Pathfinding::gpsCallback, this, _1));
	_imu = create_subscription<sensor_msgs::msg::Imu>("/wamv/sensors/imu/imu/data", 10,
				std::bind(&Pathfinding::imuCallback, this, _1));
	_publisherRangeBearing = create_publisher<ros_gz_interfaces::msg::ParamVec>("/range_bearing", 5);
}
int Pathfinding::init() {
	if (parseObstacles() == -1)
		return (-1);
	_obstaclesGraph.setNbVertices(_obstacles.size() * 4);
	generateObstaclesGraph();
	return (0);
}

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

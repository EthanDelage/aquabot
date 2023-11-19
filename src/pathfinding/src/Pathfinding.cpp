#include <fstream>
#include <iterator>

#include "Pathfinding.hpp"

using std::placeholders::_1;

Pathfinding::Pathfinding() {
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
//	for (auto node : path) {
//		std::cout << "[" << node.x << "," << node.y << "], ";
//	}
}
int Pathfinding::init() {
	if (parseObstacles() == -1)
		return (-1);
	_obstaclesGraph.setNbVertices(_obstacles.size() * 4);
	generateObstaclesGraph();
	return (0);
}

void Pathfinding::addBuoy(point_t buoyPos) {
	_buoyPos.x = buoyPos.x;
	_buoyPos.y = buoyPos.y;
	_buoyGraphIndex = _obstaclesGraph.addVertex();
	generateNodeAdjList(_buoyPos, _buoyGraphIndex, _obstaclesGraph);
}

size_t Pathfinding::addBoat(point_t boatPos, Graph& graph) {
	size_t	boatIndex;

	boatIndex = graph.addVertex();
	generateNodeAdjList(boatPos, boatIndex, graph);
	std::cout << std::endl;
	graph.printGraph();
	return (boatIndex);
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

void	Pathfinding::calculateMapPos(double latitude, double longitude) {
	double	x, y;

	x = (latitude - LATITUDE_0) / (LATITUDE_1 - LATITUDE_0);
	_boatPos.x = x * 600 - 300;
	y = (longitude - LONGITUDE_0) / (LONGITUDE_1 - LONGITUDE_0);
	_boatPos.y = y * 600 - 300;
	_gpsPing = true;
//	std::cout << "Boat: [" << _boatPos.x << ", " << _boatPos.y << "]" << std::endl;
}

int Pathfinding::parseObstacles() {
	std::string					line;
	std::ifstream				obstacleFile(OBSTACLE_FILE);

	_obstacles.clear();
	if (!obstacleFile.is_open())
		return (-1);
	while (!obstacleFile.eof()) {
		std::getline(obstacleFile, line, '\n');
		if (!line.empty())
			_obstacles.push_back(parseBoundingBox(line));
	}
	obstacleFile.close();
	return (0);
}

rectangle_t Pathfinding::parseBoundingBox(const std::string& strBoundingBox) {
	static size_t	id = 0;
	rectangle_t		boudingBox;
	char*			rest;

	boudingBox.point[0].x = strtod(strBoundingBox.c_str(), &rest);
	boudingBox.point[0].y = strtod(rest + 1, &rest);
	boudingBox.point[1].x = strtod(rest + 1, &rest);
	boudingBox.point[1].y = strtod(rest + 1, &rest);
	boudingBox.point[2].x = strtod(rest + 1, &rest);
	boudingBox.point[2].y = strtod(rest + 1, &rest);
	boudingBox.point[3].x = strtod(rest + 1, &rest);
	boudingBox.point[3].y = strtod(rest + 1, NULL);
	boudingBox.id = id;
	++id;
	return (boudingBox);
}

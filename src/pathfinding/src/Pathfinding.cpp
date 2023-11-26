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
	_allies = create_subscription<geometry_msgs::msg::PoseArray>("/wamv/ais_sensor/allies_positions", 10,
				std::bind(&Pathfinding::alliesCallback, this, _1));
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
	Graph									graph(_obstaclesGraph);
	size_t									boatIndex;
	std::vector<std::pair<size_t, double>>	reversePath;

	boatIndex = addCheckPoint(boatPos, graph);

	reversePath = djikstra(boatIndex, _buoy.graphIndex, graph);

	return (convertDjikstraToPoint(reversePath, boatIndex, _buoy.graphIndex));
}

std::vector<point_t> Pathfinding::calculatePathWithAlly(point_t boatPos, std::pair<point_t, double> ally) {
	Graph									graph(_obstaclesGraph);
	size_t									boatIndex;
	checkpoint_t							allyCheckpoint[2];
	std::vector<std::pair<size_t, double>>	reversePath;
	std::vector<point_t>					tmpPath;
	std::vector<point_t>					path;

	boatIndex = addCheckPoint(boatPos, graph);

	calculateAllyCheckpoint(allyCheckpoint, ally);

	allyCheckpoint[0].graphIndex = addCheckPoint(allyCheckpoint[0].position, graph);
	allyCheckpoint[1].graphIndex = addCheckPoint(allyCheckpoint[1].position, graph);

	if (calculateDist(boatPos, allyCheckpoint[0].position) < calculateDist(boatPos, allyCheckpoint[1].position)) {
		reversePath = djikstra(boatIndex, allyCheckpoint[0].graphIndex, graph);
		path = convertDjikstraToPoint(reversePath, boatIndex, allyCheckpoint[0].graphIndex);
		path.push_back(allyCheckpoint[1].position);
		reversePath = djikstra(allyCheckpoint[1].graphIndex, _buoy.graphIndex, graph);
		tmpPath = convertDjikstraToPoint(reversePath, allyCheckpoint[1].graphIndex, _buoy.graphIndex);
		path.insert(path.end(), tmpPath.begin(), tmpPath.end());
	} else {
		reversePath = djikstra(boatIndex, allyCheckpoint[1].graphIndex, graph);
		path = convertDjikstraToPoint(reversePath, boatIndex, allyCheckpoint[1].graphIndex);
		path.push_back(allyCheckpoint[0].position);
		reversePath = djikstra(allyCheckpoint[0].graphIndex, _buoy.graphIndex, graph);
		tmpPath = convertDjikstraToPoint(reversePath, allyCheckpoint[0].graphIndex, _buoy.graphIndex);
		path.insert(path.end(), tmpPath.begin(), tmpPath.end());
	}
	return (path);
}

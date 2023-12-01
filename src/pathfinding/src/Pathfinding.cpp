#include "Pathfinding.hpp"

#include <iterator>

using std::placeholders::_1;

Pathfinding::Pathfinding() :
	Node("pathfinding") {

	_buoyPosCalculate = false;
	_pathCalculated = false;
	_buoyPing = false;
	_gpsPing = false;
	_imuPing = false;
	_enemyPing = false;
	_state = 0;
	_scan = 0;

	if (init() == -1) {
		std::cout << "Cannot open " <<  OBSTACLE_FILE << std::endl;
		exit(1);
	}
	_pinger = create_subscription<ros_gz_interfaces::msg::ParamVec>("/wamv/sensors/acoustics/receiver/range_bearing", 10,
				std::bind(&Pathfinding::pingerCallback, this, _1));
	_perception = create_subscription<ros_gz_interfaces::msg::ParamVec>("/perception/pinger", 10,
				std::bind(&Pathfinding::perceptionCallback, this, _1));
	_phase = create_subscription<std_msgs::msg::UInt32>("/vrx/patrolandfollow/current_phase", 10,
				std::bind(&Pathfinding::phaseCallback, this, _1));
	_gps = create_subscription<sensor_msgs::msg::NavSatFix>("/wamv/sensors/gps/gps/fix", 10,
				std::bind(&Pathfinding::gpsCallback, this, _1));
	_imu = create_subscription<sensor_msgs::msg::Imu>("/wamv/sensors/imu/imu/data", 10,
				std::bind(&Pathfinding::imuCallback, this, _1));
	_allies = create_subscription<geometry_msgs::msg::PoseArray>("/wamv/ais_sensor/allies_positions", 10,
				std::bind(&Pathfinding::alliesCallback, this, _1));
	_publisherRangeBearing = create_publisher<ros_gz_interfaces::msg::ParamVec>("/navigation/pinger", 5);
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

	if (_state < FOLLOW_STATE && _enemyPing)
		return (calculatePathWithAllies(_boatPos, std::vector<std::pair<point_t, double>>()));

	_target.graphIndex = addCheckPoint(_target.position, graph);

	boatIndex = addCheckPoint(boatPos, graph);

	reversePath = djikstra(boatIndex, _target.graphIndex, graph);
	_checkpoints.clear();
	return (convertDjikstraToPoint(reversePath, boatIndex, _target.graphIndex));
}

std::vector<point_t> Pathfinding::calculatePathWithAllies(point_t boatPos, std::vector<std::pair<point_t, double>> allies) {
	std::vector<rectangle_t>					obstaclesSave;
	Graph										graphSave;
	size_t                      				boatIndex;
	std::vector<std::pair<size_t, double>>		reversePath;
	std::vector<point_t>                    	path;

	obstaclesSave = _obstacles;
	graphSave = _obstaclesGraph;
	for (auto ally : allies)
		_obstacles.push_back(calculateAllyBoundingBox(ally, _obstacles.size()));
	if (_state < FOLLOW_STATE && _enemyPing)
		_obstacles.push_back(calculateEnemyBoundingBox(_enemyPos, _obstacles.size()));

	_obstaclesGraph.setNbVertices(_obstacles.size() * 4);
	generateObstaclesGraph();

	_target.graphIndex = addCheckPoint(_target.position, _obstaclesGraph);
	boatIndex = addCheckPoint(boatPos, _obstaclesGraph);

	reversePath = djikstra(boatIndex, _target.graphIndex, _obstaclesGraph);
	path = convertDjikstraToPoint(reversePath, boatIndex, _target.graphIndex);

	for (auto node : path)
		std::cout << "[" << node.x << "," << node.y << "], ";
	std::cout << std::endl;

	_obstacles = obstaclesSave;
	_obstaclesGraph = graphSave;
	_checkpoints.clear();
	return (path);
}

#include "Pathfinding.hpp"
#include <cmath>

void Pathfinding::generateObstaclesGraph() {
	for (auto it = _obstacles.begin(); it != _obstacles.end(); ++it) {
		addObstacleAdjList(*it, std::distance(_obstacles.begin(), it));
	}
	for (size_t src = 0; src < _obstacles.size() - 1; ++src) {
		for (size_t dest = src + 1; dest < _obstacles.size(); ++dest) {
			generateAdjList(_obstacles[src], _obstacles[dest]);
		}
	}
	_obstaclesGraph.printGraph();
}

void Pathfinding::addObstacleAdjList(rectangle_t const & obstacle, size_t index) {
	if (!isHitObstacle(obstacle.point[0], obstacle.point[1], obstacle))
		_obstaclesGraph.addEdge(index * 4, index * 4 + 1, calculateDist(obstacle.point[0], obstacle.point[1]));
	if (!isHitObstacle(obstacle.point[1], obstacle.point[2], obstacle))
		_obstaclesGraph.addEdge(index * 4 + 1, index * 4 + 2, calculateDist(obstacle.point[1], obstacle.point[2]));
	if (!isHitObstacle(obstacle.point[2], obstacle.point[3], obstacle))
		_obstaclesGraph.addEdge(index * 4 + 2, index * 4 + 3, calculateDist(obstacle.point[2], obstacle.point[3]));
	if (!isHitObstacle(obstacle.point[3], obstacle.point[0], obstacle))
		_obstaclesGraph.addEdge(index * 4 + 3, index * 4, calculateDist(obstacle.point[3], obstacle.point[0]));
}

void Pathfinding::generateAdjList(const rectangle_t& lhs, const rectangle_t& rhs) {
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			if (_obstacles.size() > 2) {
				if (!isHitObstacle(lhs.point[i], rhs.point[j], lhs, rhs)
					&& !isHitItself(lhs.point[i], rhs.point[j], lhs, i)
					&& !isHitItself(lhs.point[i], rhs.point[j], rhs, j))
					_obstaclesGraph.addEdge(lhs.id * 4 + i, rhs.id * 4 + j, calculateDist(lhs.point[i], rhs.point[j]));
			} else {
				if (!isHitItself(lhs.point[i], rhs.point[j], lhs, i)
					&& !isHitItself(lhs.point[i], rhs.point[j], rhs, j))
					_obstaclesGraph.addEdge(lhs.id * 4 + i, rhs.id * 4 + j, calculateDist(lhs.point[i], rhs.point[j]));
			}
		}
	}
}

void Pathfinding::generateNodeAdjList(point_t nodePos, size_t nodeIndex, Graph& graph) {
	for (size_t i = 0; i != _obstacles.size(); ++i) {
		for (size_t j = 0; j < 4; ++j) {
			if (!isHitItself(nodePos, _obstacles[i].point[j], _obstacles[i], j)
				&& !isHitObstacle(nodePos, _obstacles[i].point[j], _obstacles[i])) {
				graph.addEdge(nodeIndex, i * 4 + j, calculateDist(nodePos, _obstacles[i].point[j]));
			}
		}
	}
}

double Pathfinding::calculateDist(point_t a, point_t b) {
	return (std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2)));
}

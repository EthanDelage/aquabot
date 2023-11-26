#include "Pathfinding.hpp"
#include <cmath>

size_t Pathfinding::addCheckPoint(point_t cpPos, Graph& graph) {
	size_t			checkpointIndex;
	checkpoint_t	cp;

	cp.position.x = cpPos.x;
	cp.position.y = cpPos.y;
	cp.graphIndex = graph.addVertex();
	generateCheckpointAdjList(cp, graph);
	std::cout << std::endl;
	graph.printGraph();

	return (checkpointIndex);
}

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

void Pathfinding::generateCheckpointAdjList(checkpoint_t cp, Graph& graph) {
	for (size_t i = 0; i != _obstacles.size(); ++i) {
		for (size_t j = 0; j < 4; ++j) {
			if (!isHitItself(cp.position, _obstacles[i].point[j], _obstacles[i], j)
				&& !isHitObstacle(cp.position, _obstacles[i].point[j], _obstacles[i]))
				graph.addEdge(cp.graphIndex, i * 4 + j, calculateDist(cp.position, _obstacles[i].point[j]));
		}
	}
	if (graph.getNbVertices() != _obstacles.size() * 4 + 1) {
		for (size_t i = 0; i != _checkpoints.size(); ++i) {
			if (cp.graphIndex != i
				&& !isHitObstacle(cp.position, _checkpoints[i].position))
				graph.addEdge(cp.graphIndex, _obstacles.size() * 4 + i,
							  calculateDist(cp.position, _checkpoints[i].position));
		}
	}
}

double Pathfinding::calculateDist(point_t a, point_t b) {
	return (std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2)));
}

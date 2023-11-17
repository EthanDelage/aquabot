/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   pathfinding.cpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: edelage <edelage@student.42lyon.fr>        +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2023/11/12 13:52:00 by edelage           #+#    #+#             */
/*   Updated: 2023/11/12 13:52:00 by edelage          ###   ########lyon.fr   */
/*                                                                            */
/* ************************************************************************** */
#include <fstream>
#include <limits>
#include <algorithm>
#include <iterator>
#include <cmath>
#include "Pathfinding.hpp"

Pathfinding::Pathfinding() :
	_obstacles(parseObstacles()),
	_obstaclesGraph(_obstacles.size() * 4) {
	generateObstaclesGraph();
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

	//implement djikstra algorithm
	reversePath = djikstra(boatIndex, _buoyGraphIndex, graph);

	current = reversePath[_buoyGraphIndex];

	path.push_front(_buoyGraphIndex);
	while (current.first != boatIndex) {
		path.push_front(current.first);
		current = reversePath[current.first];
	}
	return (convertNodeToPoint(path));
}

std::vector<std::pair<size_t, double>> Pathfinding::djikstra(size_t start, size_t end, Graph const & graph) {
	adjList_t 								adjList;
	std::vector<size_t>						visited(1, start);
	std::pair<size_t, double>				current(start, 0);
	std::vector<std::pair<size_t, double>>	path(graph.getNbVertices(), std::pair<int, double>(-1, std::numeric_limits<double>::infinity()));

	adjList = graph.getAdjList();
	path[start] = current;
	while (current.first != _buoyGraphIndex) {
		for (auto node : adjList[current.first]) {
			if (isVisited(node.first, visited) || node.first == current.first)
				continue;
			if (current.second + node.second < path[node.first].second)
				path[node.first] = std::pair<int, double>(current.first, current.second + node.second);
		}
		current = getMinNode(path, visited);
		visited.push_back(current.first);
	}
	return (path);
}

bool Pathfinding::isVisited(size_t index, const std::vector<size_t>& visited) {
	return (std::find(visited.begin(), visited.end(), index) != visited.end());
}

std::pair<size_t, double> Pathfinding::getMinNode(
	std::vector<std::pair<size_t, double>> const & path,
	std::vector<size_t> const & visited) {

	double					minDist = std::numeric_limits<double>::infinity();
	std::pair<int, double>	min;
	size_t					index = 0;

	for (auto node : path) {
		if ((node.second < minDist || minDist == std::numeric_limits<double>::infinity())
			&& !isVisited(index, visited)) {
			min.second = node.second;
			min.first = index;
			minDist = min.second;
		}
		++index;
	}
	return (min);
}

std::vector<point_t> Pathfinding::convertNodeToPoint(std::list<size_t> nodePath) {
	std::vector<point_t>	path;

	for (auto node : nodePath) {
		if (node != _buoyGraphIndex)
			path.push_back(_obstacles[node / 4].point[node % 4]);
		else
			path.push_back(_buoyPos);
	}
	return (path);
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
	_obstaclesGraph.addEdge(index * 4, index * 4 + 1, calculateDist(obstacle.point[0], obstacle.point[1]));
	_obstaclesGraph.addEdge(index * 4 + 1, index * 4 + 2, calculateDist(obstacle.point[1], obstacle.point[2]));
	_obstaclesGraph.addEdge(index * 4 + 2, index * 4 + 3, calculateDist(obstacle.point[2], obstacle.point[3]));
	_obstaclesGraph.addEdge(index * 4 + 3, index * 4, calculateDist(obstacle.point[3], obstacle.point[0]));
}

void Pathfinding::generateAdjList(const rectangle_t& lhs, const rectangle_t& rhs) {
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			if (_obstacles.size() > 2) {
				for (auto it = _obstacles.begin(); it != _obstacles.end(); ++it) {
					if (!areRectangleEqual(*it, lhs) && !areRectangleEqual(*it, rhs)) {
						if (!isHitObstacle(lhs.point[i], rhs.point[j], *it)
							&& !isHitItself(lhs.point[i], rhs.point[j], lhs, i)
							&& !isHitItself(lhs.point[i], rhs.point[j], rhs, j))
							_obstaclesGraph.addEdge(lhs.id * 4 + i, rhs.id * 4 + j, calculateDist(lhs.point[i], rhs.point[j]));
					}
				}
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
			if (!isHitItself(nodePos, _obstacles[i].point[j], _obstacles[i], j)) {
				if (_obstacles.size() > 1) {
					for (auto obstacle : _obstacles) {
						if (!areRectangleEqual(obstacle, _obstacles[i])
							&& !isHitObstacle(nodePos, _obstacles[i].point[j], obstacle))
							graph.addEdge(nodeIndex, i * 4 + j, calculateDist(nodePos, _obstacles[i].point[j]));
					}
				} else {
					graph.addEdge(nodeIndex, i * 4 + j, calculateDist(nodePos, _obstacles[i].point[j]));
				}
			}
		}
	}
}

double Pathfinding::calculateDist(point_t a, point_t b) {
	return (std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2)));
}

bool Pathfinding::areRectangleEqual(const rectangle_t& lhs, const rectangle_t& rhs) {
	return (lhs.point == rhs.point);
}

std::vector<rectangle_t> Pathfinding::parseObstacles() {
	std::vector<rectangle_t>	obstacles;
	std::string					line;
	std::ifstream				obstacleFile(OBSTACLE_FILE);

	obstacles.clear();
	if (!obstacleFile.is_open())
		throw (std::runtime_error(std::string("Cannot open ") + OBSTACLE_FILE));
	while (!obstacleFile.eof()) {
		std::getline(obstacleFile, line, '\n');
		if (!line.empty())
			obstacles.push_back(parseBoundingBox(line));
	}
	obstacleFile.close();
	return (obstacles);
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

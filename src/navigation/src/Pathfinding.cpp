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
#include <cmath>
#include "Pathfinding.hpp"

Pathfinding::Pathfinding() :
	_obstacles(parseObstacles()),
	_obstaclesGraph(_obstacles.size() * 4) {
	generateObstaclesGraph();
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

double Pathfinding::calculateDist(point_t a, point_t b) {
	return (std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2)));
}

bool Pathfinding::areRectangleEqual(const rectangle_t& lhs, const rectangle_t& rhs) {
	return (lhs.point == rhs.point);
}

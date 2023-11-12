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
	rectangle_t	boudingBox;
	char*		rest;

	boudingBox.a.x = strtod(strBoundingBox.c_str(), &rest);
	boudingBox.a.y = strtod(rest + 1, &rest);
	boudingBox.b.x = strtod(rest + 1, &rest);
	boudingBox.b.y = strtod(rest + 1, &rest);
	boudingBox.c.x = strtod(rest + 1, &rest);
	boudingBox.c.y = strtod(rest + 1, &rest);
	boudingBox.d.x = strtod(rest + 1, &rest);
	boudingBox.d.y = strtod(rest + 1, NULL);
	std::cout << "Obstacle: [" << boudingBox.b.x << ' ' << boudingBox.b.y << ", " << boudingBox.d.x << " " << boudingBox.d.y << ']' << std::endl;
	return (boudingBox);
}

void Pathfinding::generateObstaclesGraph() {
	for (auto it = _obstacles.begin(); it != _obstacles.end(); ++it) {
		addObstacleAdjList(*it, std::distance(_obstacles.begin(), it));
	}
	_obstaclesGraph.printGraph();
}

void Pathfinding::addObstacleAdjList(rectangle_t const & obstacle, size_t index) {
	_obstaclesGraph.addEdge(index * 4, index * 4 + 1, calculateDist(obstacle.a, obstacle.b));
	_obstaclesGraph.addEdge(index * 4 + 1, index * 4 + 2, calculateDist(obstacle.b, obstacle.c));
	_obstaclesGraph.addEdge(index * 4 + 2, index * 4 + 3, calculateDist(obstacle.c, obstacle.d));
	_obstaclesGraph.addEdge(index * 4 + 3, index * 4, calculateDist(obstacle.d, obstacle.a));
}

double Pathfinding::calculateDist(point_t a, point_t b) {
	return (std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2)));
}

bool Pathfinding::isHitObstacle(point_t start, point_t end, rectangle_t obstacle) {
	return (isIntersect(start, end, obstacle.a, obstacle.b)
			|| isIntersect(start, end, obstacle.b, obstacle.c)
			|| isIntersect(start, end, obstacle.c, obstacle.d)
			|| isIntersect(start, end, obstacle.d, obstacle.a));
}

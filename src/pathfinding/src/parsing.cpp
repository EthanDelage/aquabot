#include "Pathfinding.hpp"

#include <fstream>

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

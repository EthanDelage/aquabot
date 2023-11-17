/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   pathfinding.hpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: edelage <edelage@student.42lyon.fr>        +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2023/11/12 16:31:00 by edelage           #+#    #+#             */
/*   Updated: 2023/11/12 16:31:00 by edelage          ###   ########lyon.fr   */
/*                                                                            */
/* ************************************************************************** */
#ifndef PATHFINDING_HPP
# define PATHFINDING_HPP

# include "Graph.hpp"
# include "point.hpp"
# include <list>

# define OBSTACLE_FILE	"src/navigation/obstacles.txt"

typedef struct rectangle_s {
	size_t	id;
	point_t point[4];
} rectangle_t;

class Pathfinding {

private:
	std::vector<rectangle_t>	_obstacles;
	Graph						_obstaclesGraph;
	point_t						_buoyPos;
	size_t						_buoyGraphIndex;

	int 									parseObstacles();
	rectangle_t								parseBoundingBox(std::string const & strBoundingBox);
	void									generateObstaclesGraph();
	void 									addObstacleAdjList(rectangle_t const & obstacle, size_t index);
	void									generateAdjList(rectangle_t const & lhs, rectangle_t const & rhs);
	void									generateNodeAdjList(point_t nodePos, size_t nodeIndex, Graph& graph);
	std::vector<std::pair<size_t, double>>	djikstra(size_t start, size_t end, Graph const & graph);
	std::vector<point_t>					convertNodeToPoint(std::list<size_t> nodePath);
	static bool								isVisited(size_t index, std::vector<size_t> const & visited);
	static std::pair<size_t, double>		getMinNode(std::vector<std::pair<size_t, double>> const & path, std::vector<size_t> const & visited);
	static double							calculateDist(point_t a, point_t b);
	static bool								isHitObstacle(point_t const start, point_t const end, rectangle_t const & obstacle);
	static bool								isHitItself(point_t const start, point_t const end, rectangle_t const & obstacle, size_t index);
	static bool								isIntersect(point_t p1, point_t q1, point_t p2, point_t q2);
	static bool								areRectangleEqual(rectangle_t const & lhs, rectangle_t const & rhs);

public:
	Pathfinding();

	int						init();
	void					addBuoy(point_t buoyPos);
	size_t 					addBoat(point_t boatPos, Graph& graph);
	std::vector<point_t>	calculatePath(point_t boatPos);

};

#endif

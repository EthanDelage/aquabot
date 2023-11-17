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

	std::vector<rectangle_t>			parseObstacles();
	rectangle_t							parseBoundingBox(std::string const & strBoundingBox);
	void								generateObstaclesGraph();
	void 								addObstacleAdjList(rectangle_t const & obstacle, size_t index);
	void								generateAdjList(rectangle_t const & lhs, rectangle_t const & rhs);
	void								generateNodeAdjList(point_t nodePos, size_t nodeIndex, Graph& graph);
	std::vector<std::pair<int, double>>	djikstra(int start, int end, Graph const & graph);
	static bool							isVisited(int index, std::vector<int> const & visited);
	static std::pair<int, double>		getMinNode(std::vector<std::pair<int, double>> const & path, std::vector<int> const & visited, std::vector<std::pair<int, double>> const & nodeAdjList);
	static bool							haveEdge(int nodeIndex, std::vector<std::pair<int, double>> const & nodeAdjList);
	static double						calculateDist(point_t a, point_t b);
	static bool							isHitObstacle(point_t const start, point_t const end, rectangle_t const & obstacle);
	static bool							isHitItself(point_t const start, point_t const end, rectangle_t const & obstacle, size_t index);
	static bool							isIntersect(point_t p1, point_t q1, point_t p2, point_t q2);
	static bool							areRectangleEqual(rectangle_t const & lhs, rectangle_t const & rhs);

public:
	Pathfinding();

	void					addBuoy(point_t buoyPos);
	size_t 					addBoat(point_t boatPos, Graph& graph);
	std::vector<point_t>	calculatePath(point_t boatPos);

};

#endif

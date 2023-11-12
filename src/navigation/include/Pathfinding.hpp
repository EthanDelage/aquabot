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

# define OBSTACLE_FILE	"src/navigation/obstacles.txt"

typedef struct point_s {
	double	x;
	double	y;
} point_t;

typedef struct rectangle_s {
	point_t a;
	point_t	b;
	point_t c;
	point_t d;
} rectangle_t;

class Pathfinding {

private:
	std::vector<rectangle_t>	_obstacles;
	Graph						_obstaclesGraph;

	std::vector<rectangle_t>	parseObstacles();
	rectangle_t					parseBoundingBox(std::string const & strBoundingBox);
	void						generateObstaclesGraph();
	void 						addObstacleAdjList(rectangle_t const & obstacle, size_t index);
	static double				calculateDist(point_t a, point_t b);
	static bool					isHitObstacle(point_t start, point_t end, rectangle_t obstacle);
	static bool					isIntersect(point_t p1, point_t q1, point_t p2, point_t q2);

public:
	Pathfinding();

};

#endif

/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   segmentCollision.cpp                               :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: edelage <edelage@student.42lyon.fr>        +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2023/11/12 18:52:00 by edelage           #+#    #+#             */
/*   Updated: 2023/11/12 18:52:00 by edelage          ###   ########lyon.fr   */
/*                                                                            */
/* ************************************************************************** */
#include "Pathfinding.hpp"
#include <cmath>

rectangle_t Pathfinding::calculateAllyBoundingBox(std::pair<point_t, double> const & ally, size_t id) {
	rectangle_t    boundingBox;

	boundingBox.point[0].x = (ally.first.x - 20 * std::cos(ally.second))
							 + 10 * std::cos(ally.second - M_PI_2);
	boundingBox.point[0].y = (ally.first.y - 20 * std::sin(ally.second))
							 + 10 * std::sin(ally.second - M_PI_2);
	boundingBox.point[1].x = (ally.first.x - 20 * std::cos(ally.second))
							 + 10 * std::cos(ally.second + M_PI_2);
	boundingBox.point[1].y = (ally.first.y - 20 * std::sin(ally.second))
							 + 10 * std::sin(ally.second + M_PI_2);
	boundingBox.point[2].x = (ally.first.x + 45 * std::cos(ally.second))
							 + 25 * std::cos(ally.second + M_PI_2);
	boundingBox.point[2].y = (ally.first.y + 45 * std::sin(ally.second))
							 + 25 * std::sin(ally.second + M_PI_2);
	boundingBox.point[3].x = (ally.first.x + 45 * std::cos(ally.second))
							 + 25 * std::cos(ally.second - M_PI_2);
	boundingBox.point[3].y = (ally.first.y + 45 * std::sin(ally.second))
							 + 25 * std::sin(ally.second - M_PI_2);
	boundingBox.id = id;
	return (boundingBox);
}

bool Pathfinding::isHitObstacle(point_t const start, point_t const end, rectangle_t const & lhs, rectangle_t const & rhs) {
	for (auto obstacle : _obstacles) {
		if (!(areRectangleEqual(lhs, obstacle) || areRectangleEqual(rhs, obstacle))) {
			if (isIntersect(start, end, obstacle.point[0], obstacle.point[1])
				|| isIntersect(start, end, obstacle.point[1], obstacle.point[2])
				|| isIntersect(start, end, obstacle.point[2], obstacle.point[3])
				|| isIntersect(start, end, obstacle.point[3], obstacle.point[0]))
				return (true);
		}
	}
	return (false);
}

bool Pathfinding::isHitObstacle(point_t const start, point_t const end, rectangle_t const & dest) {
	for (auto obstacle : _obstacles) {
		if (!areRectangleEqual(dest, obstacle)) {
			if (isIntersect(start, end, obstacle.point[0], obstacle.point[1])
				|| isIntersect(start, end, obstacle.point[1], obstacle.point[2])
				|| isIntersect(start, end, obstacle.point[2], obstacle.point[3])
				|| isIntersect(start, end, obstacle.point[3], obstacle.point[0]))
				return (true);
		}
	}
	return (false);
}

bool Pathfinding::isHitObstacle(point_t const start, point_t const end) {
	for (auto obstacle : _obstacles) {
		if (isIntersect(start, end, obstacle.point[0], obstacle.point[1])
			|| isIntersect(start, end, obstacle.point[1], obstacle.point[2])
			|| isIntersect(start, end, obstacle.point[2], obstacle.point[3])
			|| isIntersect(start, end, obstacle.point[3], obstacle.point[0]))
			return (true);
	}
	return (false);
}

bool Pathfinding::isHitItself(const point_t start, const point_t end, const rectangle_t& obstacle, size_t index) {
	return (isIntersect(start, end, obstacle.point[(index + 1) % 4], obstacle.point[(index + 2) % 4]))
		|| isIntersect(start, end, obstacle.point[(index + 2) % 4], obstacle.point[(index + 3) % 4]);
}

bool Pathfinding::isIntersect(point_t a1, point_t a2, point_t b1, point_t b2) {
	if (a2.x < a1.x) {
		std::swap(a1.x, a2.x);
		std::swap(a1.y, a2.y);
	}
	if (b2.x < b1.x) {
		std::swap(b1.x, b2.x);
		std::swap(b1.y, b2.y);
	}

	const point_t points[4] = {{a1.x, a1.y}, {a2.x, a2.y}, \
        {b1.x, b1.y}, {b2.x, b2.y}};
	const int            o1 = orientation(points[0], points[1], points[2]);
	const int            o2 = orientation(points[0], points[1], points[3]);
	const int            o3 = orientation(points[2], points[3], points[0]);
	const int            o4 = orientation(points[2], points[3], points[1]);

	if (o1 != o2 && o3 != o4)
		return (true);
	if (o1 == 0 && onSegment(points[0], points[2], points[1]))
		return (true);
	if (o2 == 0 && onSegment(points[0], points[3], points[2]))
		return (true);
	if (o3 == 0 && onSegment(points[2], points[0], points[3]))
		return (true);
	if (o4 == 0 && onSegment(points[2], points[1], points[3]))
		return (true);
	return (false);
}

int    Pathfinding::orientation(point_t p, point_t q, point_t r) {
	const double    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if (fabs(val) < 1e-9)
		return (0);
	if (val > 0)
		return (1);
	return (2);
}

bool    Pathfinding::onSegment(point_t p, point_t q, point_t r) {
	return (q.x <= fmax(p.x, r.x) && q.x >= fmin(p.x, r.x)
			&& q.y <= fmax(p.y, r.y) && q.y >= fmin(p.y, r.y));
}

bool Pathfinding::areRectangleEqual(const rectangle_t& lhs, const rectangle_t& rhs) {
	return (lhs.point[0].x == rhs.point[0].x && lhs.point[0].y == rhs.point[0].y
			&& lhs.point[1].x == rhs.point[1].x && lhs.point[1].y == rhs.point[1].y
			&& lhs.point[2].x == rhs.point[2].x && lhs.point[2].y == rhs.point[2].y
			&& lhs.point[3].x == rhs.point[3].x && lhs.point[3].y == rhs.point[3].y);
}

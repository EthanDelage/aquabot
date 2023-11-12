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
# include "Pathfinding.hpp"

static int orientation(point_t p, point_t q, point_t r);
static bool onSegment(point_t p, point_t q, point_t r);

bool Pathfinding::isHitObstacle(point_t const start, point_t const end, rectangle_t const & obstacle) {
	return (isIntersect(start, end, obstacle.point[0], obstacle.point[1])
		|| isIntersect(start, end, obstacle.point[1], obstacle.point[2])
		|| isIntersect(start, end, obstacle.point[2], obstacle.point[3])
		|| isIntersect(start, end, obstacle.point[3], obstacle.point[0]));
}

bool Pathfinding::isHitItself(const point_t start, const point_t end, const rectangle_t& obstacle, size_t index) {
	return (isIntersect(start, end, obstacle.point[(index + 1) % 4], obstacle.point[(index + 2) % 4]))
		|| isIntersect(start, end, obstacle.point[(index + 2) % 4], obstacle.point[(index + 3) % 4]);
}

// Vérifie si deux segments se coupent
bool Pathfinding::isIntersect(point_t p1, point_t q1, point_t p2, point_t q2) {
	// Trouver les quatre orientations nécessaires pour l'intersection
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// Cas général
	if (o1 != o2 && o3 != o4)
		return true;

	// Cas spéciaux

	// p1, q1 et p2 sont collinéaires et p2 se trouve sur le segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 et q2 sont collinéaires et q2 se trouve sur le segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 et p1 sont collinéaires et p1 se trouve sur le segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 et q1 sont collinéaires et q1 se trouve sur le segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	// Aucun cas d'intersection
	return false;
}

// Fonction pour trouver l'orientation de triplet (p, q, r).
// La fonction retourne les valeurs suivantes :
// 0 : Collinéaire
// 1 : Dans le sens des aiguilles d'une montre
// 2 : Dans le sens contraire des aiguilles d'une montre
static int orientation(point_t p, point_t q, point_t r) {
	int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0;  // Collinéaire
	return (val > 0) ? 1 : 2;  // Sens horaire ou antihoraire
}


// Vérifie si q se trouve sur le segment prédéfini
static bool onSegment(point_t p, point_t q, point_t r) {
	if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
		q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
		return true;
	return false;
}
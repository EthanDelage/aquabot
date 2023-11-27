#include "Pathfinding.hpp"

#include <cmath>

void Pathfinding::addBuoy() {
	calculateBuoyPos();
	std::cout << "Buoy pos: [" << _buoy.position.x << ", " << _buoy.position.y << "]" << std::endl;
	_buoyPosCalculate = true;
	_buoy.graphIndex = addCheckPoint(_buoy.position, _obstaclesGraph);
	generateCheckpointAdjList(_buoy, _obstaclesGraph);
}

void Pathfinding::calculateBuoyPos() {
	double buoyOrientation = convertToMinusPiPi(_orientation + _buoyBearing);
	_buoy.position.x = _buoyRange * std::cos(buoyOrientation) + _boatPos.x;
	_buoy.position.y = _buoyRange * std::sin(buoyOrientation) + _boatPos.y;
}

double Pathfinding::convertToMinusPiPi(double angleRadians) {
	while (angleRadians <= -M_PI) {
		angleRadians += 2.0 * M_PI;
	}
	while (angleRadians > M_PI) {
		angleRadians -= 2.0 * M_PI;
	}
	return (angleRadians);
}

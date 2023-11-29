#include "Pathfinding.hpp"

#include <cmath>

void Pathfinding::addBuoy() {
	calculateBuoyPos();
	std::cout << "Buoy position: [" << _target.position.x
			  << ", " << _target.position.y << "]" << std::endl;
	_targetDesiredRange = MAX_BUOY_RANGE;
	_buoyPosCalculate = true;
	_target.graphIndex = addCheckPoint(_target.position, _obstaclesGraph);
	generateCheckpointAdjList(_target, _obstaclesGraph);
}

void Pathfinding::calculateBuoyPos() {
	double targetOrientation= convertToMinusPiPi(_orientation + _targetBearing);
	_target.position.x = _targetRange * std::cos(targetOrientation) + _boatPos.x;
	_target.position.y = _targetRange * std::sin(targetOrientation) + _boatPos.y;
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

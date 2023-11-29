#include "Perception.hpp"

# define LONGITUDE_0			(-4.980343472857843)
# define LATITUDE_0				48.043601874279716
# define LONGITUDE_1			(-4.9722961068569775)
# define LATITUDE_1				48.04899798353722

void	Perception::calculateMapPos(double latitude, double longitude) {
	double	x, y;

	x = (longitude - LONGITUDE_0) / (LONGITUDE_1 - LONGITUDE_0);
	_boatMapPos[0] = x * 600 - 300;
	y = (latitude - LATITUDE_0) / (LATITUDE_1 - LATITUDE_0);
	_boatMapPos[1] = y * 600 - 300;
}

void Perception::calculateYaw(const geometry_msgs::msg::Quaternion& orientation) {
	double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
	double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
	_boatOrientation = std::atan2(siny_cosp, cosy_cosp);
}

void Perception::calculateEnemyPos() {
	double enemyOrientation = convertToMinusPiPi(_boatOrientation + _enemyBearing);
	_enemyMapPos[0] = (_enemyRangeMin + 3) * std::cos(enemyOrientation) + _boatMapPos[0];
	_enemyMapPos[1] = (_enemyRangeMin + 3) * std::sin(enemyOrientation) + _boatMapPos[1];
	_enemyGPSPos = convertMapPosToGPS(_enemyMapPos);
}

double Perception::convertToMinusPiPi(double angleRadians) {
	while (angleRadians <= -M_PI) {
		angleRadians += 2.0 * M_PI;
	}
	while (angleRadians > M_PI) {
		angleRadians -= 2.0 * M_PI;
	}
	return (angleRadians);
}

Eigen::Vector2d Perception::convertMapPosToGPS(Eigen::Vector2d mapPos) {
	Eigen::Vector2d gpsPos;

	gpsPos[0] = (mapPos[0] + 300) / 600;
	gpsPos[0] = gpsPos[0] * (LONGITUDE_1 - LONGITUDE_0) + LONGITUDE_0;
	gpsPos[1] = (mapPos[1] + 300) / 600;
	gpsPos[1] = gpsPos[1] * (LATITUDE_1 - LATITUDE_0) + LATITUDE_0;
	return (gpsPos);
}
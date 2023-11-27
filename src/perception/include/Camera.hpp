#ifndef AQUABOT_CAMERA_HPP
# define AQUABOT_CAMERA_HPP

# include <Eigen/Dense>

# include "Lidar.hpp"

typedef struct point_s point_t;

class LidarPoint;

class Camera {

public:
	Camera(Eigen::Matrix<double, 3, 4> projectionMatrix, float horizontalFov, std::pair<int, int> resolution);

	class projectionException : public std::exception {};

	[[nodiscard]] point_t projectLidarPoint(const LidarPoint& lidarPoint) const;
	[[nodiscard]] Eigen::Vector3d project3DTo2D(const Eigen::Vector3d& point3) const;

private:
	Eigen::Matrix<double, 3, 4>	_projectionMatrix;
	Eigen::Matrix4d				_lidarTransformationMatrix;
	float						_horizontalFov;
	int							_width;
	int							_height;

	[[nodiscard]] bool isValidPixel(point_t point) const;

};


#endif
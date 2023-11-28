#include "Camera.hpp"
#include <cmath>
#include <Eigen/Dense>

Camera::Camera() {}

Camera::Camera(Eigen::Matrix<double, 3, 4> projectionMatrix, double horizontalFov, std::pair<int, int> resolution):
                _projectionMatrix(projectionMatrix),
				_horizontalFov(horizontalFov),
				_width(resolution.first),
				_height(resolution.second) {
	Eigen::Vector3d lidarTranslationMatrix(0., 0., 0.);
	Eigen::Matrix3d lidarRotationMatrix;
	lidarRotationMatrix = Eigen::AngleAxisd(0.26, Eigen::Vector3d::UnitX())
						* Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY())
						* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

	_lidarTransformationMatrix = Eigen::Matrix4d::Identity();
	_lidarTransformationMatrix.block<3, 3>(0, 0) = lidarRotationMatrix;
	_lidarTransformationMatrix.block<3, 1>(0, 3) = lidarTranslationMatrix;
//	_lidarTransformationMatrix.row(3) << 0.0, 0.0, 0.0, 1.0;
}

point_t Camera::projectLidarPoint(const LidarPoint& lidarPoint) const {
	point_t point;

	if (!lidarPoint.position.allFinite()) {
		throw projectionException();
	}
	Eigen::Vector4d point3dHomogenous = lidarPoint.position.homogeneous();
	point3dHomogenous = _lidarTransformationMatrix * point3dHomogenous;
	Eigen::Vector3d point2dHomogenous = project3DTo2D(point3dHomogenous.head<3>() / point3dHomogenous[3]);
	if (point2dHomogenous[2] >= 0) {
		throw projectionException();
	}
	point.x = static_cast<int>(point2dHomogenous[0] / point2dHomogenous[2]);
	point.y = static_cast<int>(point2dHomogenous[1] / point2dHomogenous[2]);
	if (!isValidPixel(point)) {
		throw projectionException();
	}
	return (point);
}

Eigen::Vector3d Camera::project3DTo2D(const Eigen::Vector3d& point3) const {
	return (_projectionMatrix * point3.homogeneous());
}

bool Camera::isValidPixel(point_t pixel) const {
	return ((0 <= pixel.x && pixel.x < _width) && (0 <= pixel.y && pixel.y < _height));
}

double Camera::getHorizontalFov() const {
	return _horizontalFov;
}

int Camera::getWidth() const {
	return _width;
}

int Camera::getHeight() const {
	return _height;
}
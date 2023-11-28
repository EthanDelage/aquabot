#include "Lidar.hpp"

#include <thread>
#include <mutex>

bool operator==(const point_t& lhs, const point_t& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

Lidar::Lidar() {
	_points.reserve(30000);
	_visiblePoints.reserve(5000);
};

LidarPoint::LidarPoint() = default;

std::vector<LidarPoint> Lidar::getVisiblePoints() {return (_visiblePoints);}

double LidarPoint::getDistance() const {
	Eigen::Vector3d translation(-1.6, 0., -2.);
	Eigen::Vector3d correctedPosition = position + translation;
	return correctedPosition.norm();
}

void Lidar::parsePoints(sensor_msgs::msg::PointCloud2::SharedPtr& pointCloud) {
	std::vector<LidarPoint>	points;
	LidarPoint				currentPoint;

	uint8_t *buffer = pointCloud->data.data();

	size_t	point_xoffset = pointCloud->fields[0].offset;
	size_t	point_yoffset = pointCloud->fields[1].offset;
	size_t	point_zoffset = pointCloud->fields[2].offset;
	size_t	intensity_offset = pointCloud->fields[3].offset;
	size_t	ring_offset = pointCloud->fields[4].offset;
	size_t	point_step = pointCloud->point_step;
	size_t	bufferSize = pointCloud->row_step * pointCloud->height;
	_points.clear();
	_points.reserve(30000);
	for (size_t i = 0; i < bufferSize; i += point_step) {
		currentPoint.position[0] = *reinterpret_cast<float*>(buffer + i + point_xoffset);
		currentPoint.position[1] = *reinterpret_cast<float*>(buffer + i + point_yoffset);
		currentPoint.position[2] = *reinterpret_cast<float*>(buffer + i + point_zoffset);
		currentPoint.intensity = *reinterpret_cast<float*>(buffer + i + intensity_offset);
		currentPoint.ring = *reinterpret_cast<uint16_t*>(buffer + i + ring_offset);
		_points.push_back(currentPoint);
	}
}

void Lidar::setVisiblePoints(const Camera& camera) {
	_visiblePoints.clear();
	_visiblePoints.reserve(5000);
	const int num_threads = std::thread::hardware_concurrency();
	std::vector<std::thread> threads;
	std::mutex mutex;

	auto processPoints = [this, &camera, &mutex](auto start, auto end) {
		for (auto i = start; i != end; ++i) {
			try {
				LidarPoint& point = _points[i];
				point.imagePosition = camera.projectLidarPoint(point);

				std::lock_guard<std::mutex> lock(mutex);
				_visiblePoints.push_back(point);
			} catch (const std::exception& e) {}
		}
	};

	const size_t chunk_size = _points.size() / num_threads;
	size_t start = 0;
	for (int i = 0; i < num_threads - 1; ++i) {
		size_t end = start + chunk_size;
		threads.emplace_back(processPoints, start, end);
		start = end;
	}

	threads.emplace_back(processPoints, start, _points.size());

	for (auto& thread : threads) {
		thread.join();
	}
}
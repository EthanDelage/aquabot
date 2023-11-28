#ifndef AQUABOT_LIDAR_HPP
# define AQUABOT_LIDAR_HPP

# include <vector>
# include <Eigen/Dense>
# include <opencv2/opencv.hpp>

# include "rclcpp/rclcpp.hpp"
# include "sensor_msgs/msg/point_cloud2.hpp"
# include "Camera.hpp"

class Camera;

typedef struct point_s {
	int x;
	int	y;
} point_t;

bool operator==(const point_t& lhs, const point_t& rhs);

class LidarPoint {

public:
		LidarPoint();

        Eigen::Vector3d position;
        point_t			imagePosition;
        float			intensity;
        uint16_t		ring;

		double getDistance();
};

class Lidar {

public:
	Lidar();

	std::vector<LidarPoint> getVisiblePoints();

	void parsePoints(sensor_msgs::msg::PointCloud2::SharedPtr& pointCloud);
	void setVisiblePoints(const Camera& camera);

private:
	std::vector<LidarPoint> _points;
	std::vector<LidarPoint> _visiblePoints;
};

#endif
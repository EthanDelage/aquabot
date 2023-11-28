#ifndef AQUABOT_PERCEPTION_HPP
# define AQUABOT_PERCEPTION_HPP

# include <memory>
# include <opencv2/opencv.hpp>
# include "rclcpp/rclcpp.hpp"
# include "sensor_msgs/msg/image.hpp"
# include "sensor_msgs/msg/camera_info.hpp"
# include "sensor_msgs/msg/point_cloud2.hpp"
# include "ros_gz_interfaces/msg/param_vec.hpp"
# include "Camera.hpp"

# define CAMERA_FOV			1.3962634
# define LIDAR_MAX_RANGE	130
# define DESIRED_RANGE		30

class Perception : public rclcpp::Node {

public:
	Perception();

private:
	cv::Mat					_image;
	Camera					_camera;
	Lidar					_lidar;
	std::vector<point_t>	_enemyPixels;
	double 					_enemyBearing;
	double 					_enemyRange;
	cv::Scalar				_rgbLowerRed{65, 6, 5};
    cv::Scalar				_rgbUpperRed{110, 16, 15};
    cv::Scalar				_rgbLowerGreen{7, 50, 0};
    cv::Scalar				_rgbUpperGreen{28, 81, 10};
	bool					_imageReceived;
	bool					_cameraReceived;
	bool					_enemyFound;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr		_imageSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr	_cameraSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr	_pointCloudSubscriber;
	rclcpp::Publisher<ros_gz_interfaces::msg::ParamVec>::SharedPtr	_navigationPublisher;

	void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
	void cameraCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg);
	void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

	void detectRedBoat();
	static bool isEnemyContour(const std::vector<cv::Point>& redContour, const std::vector<std::vector<cv::Point>>& greenContours);
	static bool isContourInsideRect(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2);
	static cv::Rect maxBoundingBox(const cv::Rect& largestBoundingBox, const cv::Rect& currentBoundingBox);
	void setEnemyPixels(const cv::Rect& largestBoundingBox, cv::Mat& rgbRedMask);
	void setEnemyBearing(const cv::Rect enemyBoundingBox);
	double calculateEnemyRange();
	void drawLidarPointsInImage();
	void publishNavigation();
};

#endif
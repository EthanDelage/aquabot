#ifndef AQUABOT_PERCEPTION_HPP
# define AQUABOT_PERCEPTION_HPP

# include <memory>
# include <opencv2/opencv.hpp>
# include "rclcpp/rclcpp.hpp"
# include "sensor_msgs/msg/image.hpp"
# include "sensor_msgs/msg/camera_info.hpp"
# include "sensor_msgs/msg/point_cloud2.hpp"
# include "sensor_msgs/msg/imu.hpp"
# include "sensor_msgs/msg/nav_sat_fix.hpp"
# include "ros_gz_interfaces/msg/param_vec.hpp"
# include "geometry_msgs/msg/pose_stamped.hpp"
# include "std_msgs/msg/u_int32.hpp"
# include "Camera.hpp"

# define CAMERA_FOV			1.3962634
# define LIDAR_MAX_RANGE	130
# define DESIRED_RANGE		30
# define ALLOW_ALERT_ERROR_THRESHOLD 30
# define FOLLOW_STATE			2

class Perception : public rclcpp::Node {

public:
	Perception();

private:
	cv::Mat					_image;
	Camera					_camera;
	Lidar					_lidar;
	std::vector<point_t>	_enemyPixels;
	double 					_enemyBearing;
	double 					_enemyRangeMin;
	double					_boatOrientation;
	cv::Scalar				_rgbLowerRed{65, 6, 5};
    cv::Scalar				_rgbUpperRed{110, 16, 15};
    cv::Scalar				_rgbLowerGreen{7, 50, 0};
    cv::Scalar				_rgbUpperGreen{28, 81, 10};
	bool					_imageReceived;
	bool					_cameraReceived;
	bool					_enemyFound;
	bool					_gpsPing;
	bool					_imuPing;
	bool					_lidarPing;
	bool					_calculatedEnemyPos;
	Eigen::Vector2d			_boatMapPos;
	Eigen::Vector2d 		_enemyMapPos;
	Eigen::Vector2d			_enemyGPSPos;
	std::vector<double>		_rangeHistory;
	uint32_t 				_state;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr		_imageSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr	_cameraSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr	_pointCloudSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr			_imuSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr	_gpsSubscriber;
	rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr			_stateSubscriber;
	rclcpp::Publisher<ros_gz_interfaces::msg::ParamVec>::SharedPtr	_perceptionPublisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr	_alertPublisher;

	void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
	void cameraCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg);
	void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
	void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);
	void gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void stateCallback(std_msgs::msg::UInt32::SharedPtr msg);

	//	computePosition
	void calculateMapPos(double latitude, double longitude);
	void calculateYaw(const geometry_msgs::msg::Quaternion& orientation);
	void calculateEnemyPos();
	double convertToMinusPiPi(double angleRadians);
	static Eigen::Vector2d convertMapPosToGPS(Eigen::Vector2d mapPos);

	void detectRedBoat();
	static bool isEnemyContour(const std::vector<cv::Point>& redContour, const std::vector<std::vector<cv::Point>>& greenContours);
	static bool isContourInsideRect(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2);
	static cv::Rect maxBoundingBox(const cv::Rect& largestBoundingBox, const cv::Rect& currentBoundingBox);
	void setEnemyPixels(const cv::Rect& largestBoundingBox, cv::Mat& rgbRedMask);
	void setEnemyBearing(const cv::Rect enemyBoundingBox);
	double calculateEnemyRange();
	void drawLidarPointsInImage();
	void publishPathfinding();
	void publishAlert();
};

#endif
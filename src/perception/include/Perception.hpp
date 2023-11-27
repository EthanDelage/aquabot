#ifndef AQUABOT_PERCEPTION_HPP
# define AQUABOT_PERCEPTION_HPP

# include <opencv2/opencv.hpp>
# include "rclcpp/rclcpp.hpp"
# include "sensor_msgs/msg/image.hpp"
# include "sensor_msgs/msg/camera_info.hpp"
# include "sensor_msgs/msg/point_cloud2.hpp"

class Perception : public rclcpp::Node {

public:
	Perception();

private:
	cv::Mat								_image;
	std::vector<std::pair<int, int> >	_enemyPixels;
	cv::Scalar							_rgbLowerRed{65, 6, 5};
    cv::Scalar							_rgbUpperRed{110, 16, 15};
    cv::Scalar							_rgbLowerGreen{7, 50, 0};
    cv::Scalar							_rgbUpperGreen{28, 81, 10};

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr		_imageSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr	_cameraInfoSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr	_pointCloudSubscriber;

	void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
	//void CameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg);
	void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

	void detectRedBoat();
	static bool isEnemyContour(const std::vector<cv::Point>& redContour, const std::vector<std::vector<cv::Point>>& greenContours);
	static bool isContourInsideRect(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2);
	static cv::Rect* maxBoundingBox(cv::Rect* largestBoundingBox, cv::Rect* currentBoundingBox);
	void setEnemyPixels(const cv::Rect* largestBoundingBox, cv::Mat& rgbRedMask);
};

#endif
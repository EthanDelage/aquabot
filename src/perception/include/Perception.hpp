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
	cv::Mat		_image;
	cv::Scalar	_rgbLowerRed{5, 6, 65};
    cv::Scalar	_rgbUpperRed{15, 16, 110};
    cv::Scalar	_rgbLowerGreen{0, 50, 7};
    cv::Scalar	_rgbUpperGreen{10, 81, 28};

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr		_imageSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr	_cameraInfoSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr	_pointCloudSubscriber;

	void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
	void detectRedBoat();
	static bool isEnemyContour(const std::vector<cv::Point>& redContour, const std::vector<std::vector<cv::Point>>& greenContours);
	static bool isContourInsideRect(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2);
	static cv::Rect const * maxBoundingBox(cv::Rect const * largestBoundingBox, cv::Rect const * currentBoundingBox);
	//void CameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg);
	//void PointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

};

#endif
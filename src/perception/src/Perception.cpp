#include <cv_bridge/cv_bridge.h>

#include "Perception.hpp"
#include "Lidar.hpp"

using std::placeholders::_1;

Perception::Perception(): Node("perception") {
	_imageSubscriber = create_subscription<sensor_msgs::msg::Image>(
			"/wamv/sensors/cameras/main_camera_sensor/image_raw",
			10,
			std::bind(&Perception::imageCallback, this, _1));
	_pointCloudSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>(
			"/wamv/sensors/lidars/lidar_wamv_sensor/points",
	        10,
			std::bind(&Perception::pointCloudCallback, this, _1));

}

void Perception::imageCallback(sensor_msgs::msg::Image::SharedPtr msg) {
	try {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::cvtColor(cv_ptr->image, _image, cv::COLOR_BGR2RGB);

		detectRedBoat();

		cv::imshow("Image", _image);

		cv::waitKey(1);
	} catch (const std::exception& e){
		RCLCPP_ERROR(this->get_logger(), "imageCallback() exception: %s", e.what());
	}
}

void Perception::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
	Lidar lidar;

	lidar.parsePoints(msg);
}

void Perception::detectRedBoat() {
	cv::Mat rgbRedMask, rgbGreenMask;
	std::vector<std::vector<cv::Point>> redContours, greenContours;

    cv::inRange(_image, _rgbLowerRed, _rgbUpperRed, rgbRedMask);
    cv::inRange(_image, _rgbLowerGreen, _rgbUpperGreen, rgbGreenMask);

	cv::findContours(rgbRedMask, redContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	cv::findContours(rgbGreenMask, greenContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	cv::Rect*	largestBoundingRect = nullptr;

	for (const auto& contour : redContours) {
	    if (isEnemyContour(contour, greenContours)) {
	        cv::Rect boundingRect = cv::boundingRect(contour);
			largestBoundingRect = maxBoundingBox(largestBoundingRect, &boundingRect);
	    }
	}
	if (largestBoundingRect != nullptr) {
		cv::rectangle(
				_image,
				cv::Point(largestBoundingRect->x, largestBoundingRect->y),
				cv::Point(largestBoundingRect->x + largestBoundingRect->width, largestBoundingRect->y + largestBoundingRect->height),
				cv::Scalar(0, 255, 0), 1);
	}
	setEnemyPixels(largestBoundingRect, rgbRedMask);
}

bool Perception::isEnemyContour(const std::vector<cv::Point>& redContour, const std::vector<std::vector<cv::Point>>& greenContours) {
	for (const auto &greenContour: greenContours) {
		if (isContourInsideRect(redContour, greenContour)) {
			return false;
		}
	}
	return true;
}

bool Perception::isContourInsideRect(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2) {
	cv::Rect rect1 = cv::boundingRect(contour1);
	cv::Rect rect2 = cv::boundingRect(contour2);

	int x1 = rect1.x, y1 = rect1.y, w1 = rect1.width, h1 = rect1.height;
	int x2 = rect2.x, y2 = rect2.y, w2 = rect2.width, h2 = rect2.height;

	return ((x2 <= x1 && x1 + w1 <= x2 + w2) && (y2 <= y1 && y1 + h1 <= y2 + h2));
}

cv::Rect* Perception::maxBoundingBox(cv::Rect* largestBoundingBox, cv::Rect* currentBoundingBox) {
	if (largestBoundingBox == nullptr || currentBoundingBox->width * currentBoundingBox->height > largestBoundingBox->width * largestBoundingBox->height) {
		return (currentBoundingBox);
	}
	return (largestBoundingBox);
}

void Perception::setEnemyPixels(const cv::Rect* largestBoundingBox, cv::Mat& rgbRedMask) {
	if (largestBoundingBox != nullptr) {
		int x = largestBoundingBox->x;
		int y = largestBoundingBox->y;
		int w = largestBoundingBox->width;
		int h = largestBoundingBox->height;

		_enemyPixels.clear();

		for (int i = x; i < x + w; ++i) {
			for (int j = y; j < y + h; ++j) {
				if (rgbRedMask.at<uchar>(j, i) == 255) {
					_enemyPixels.push_back(std::make_pair(i, j));
				}
			}
		}
	}
}

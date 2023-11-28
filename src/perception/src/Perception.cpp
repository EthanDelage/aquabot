#include <memory>
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
	_cameraSubscriber = create_subscription<sensor_msgs::msg::CameraInfo>(
			"/wamv/sensors/cameras/main_camera_sensor/camera_info",
			10,
			std::bind(&Perception::cameraCallback, this, _1));
	_navigationPublisher = create_publisher<ros_gz_interfaces::msg::ParamVec>(
			"/range_bearing", 10);






}

void Perception::imageCallback(sensor_msgs::msg::Image::SharedPtr msg) {
	try {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		_image = cv_ptr->image;
		detectRedBoat();
		_imageReceived = true;

	} catch (const std::exception& e){
		RCLCPP_ERROR(this->get_logger(), "imageCallback() exception: %s", e.what());
	}
}

void Perception::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
	auto start = std::chrono::high_resolution_clock::now();
	if (_imageReceived && _cameraReceived) {
		_lidar.parsePoints(msg);
		_lidar.setVisiblePoints(_camera);
		drawLidarPointsInImage();
		cv::imshow("Image", _image);
		cv::waitKey(1);
	}
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//	std::cout << "Time in ms: " << duration.count() << std::endl;
}

void Perception::cameraCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg) {
	if (!_cameraReceived) {
		std::pair<int, int> resolution(msg->width, msg->height);
		Eigen::Matrix<double, 3, 4>	projectionMatrix;
		projectionMatrix(0, 0) = msg->p[0];
		projectionMatrix(0, 1) = msg->p[1];
		projectionMatrix(0, 2) = msg->p[2];
		projectionMatrix(0, 3) = msg->p[3];
		projectionMatrix(1, 0) = msg->p[4];
		projectionMatrix(1, 1) = msg->p[5];
		projectionMatrix(1, 2) = msg->p[6];
		projectionMatrix(1, 3) = msg->p[7];
		projectionMatrix(2, 0) = msg->p[8];
		projectionMatrix(2, 1) = msg->p[9];
		projectionMatrix(2, 2) = msg->p[10];
		projectionMatrix(2, 3) = msg->p[11];
		_camera = Camera(projectionMatrix, CAMERA_FOV, resolution);
		_cameraReceived = true;
	}
}

void Perception::detectRedBoat() {
	cv::Mat rgbImage;
	cv::Mat rgbRedMask, rgbGreenMask;
	std::vector<std::vector<cv::Point>> redContours, greenContours;

	cv::cvtColor(_image, rgbImage, cv::COLOR_BGR2RGB);
    cv::inRange(rgbImage, _rgbLowerRed, _rgbUpperRed, rgbRedMask);
    cv::inRange(rgbImage, _rgbLowerGreen, _rgbUpperGreen, rgbGreenMask);

	cv::findContours(rgbRedMask, redContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	cv::findContours(rgbGreenMask, greenContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	cv::Rect	largestBoundingRect;
	bool		foundLargestBoundingRect = false;

	for (const auto& contour : redContours) {
	    if (isEnemyContour(contour, greenContours)) {
	        cv::Rect boundingRect = cv::boundingRect(contour);
			if (!foundLargestBoundingRect) {
				largestBoundingRect = boundingRect;
				foundLargestBoundingRect = true;
			} else {
				largestBoundingRect = maxBoundingBox(largestBoundingRect, boundingRect);
			}

	    }
	}
	if (foundLargestBoundingRect) {
		cv::rectangle(
				_image,
				cv::Point(largestBoundingRect.x, largestBoundingRect.y),
				cv::Point(largestBoundingRect.x + largestBoundingRect.width, largestBoundingRect.y + largestBoundingRect.height),
				cv::Scalar(0, 255, 0), 1);
		setEnemyPixels(largestBoundingRect, rgbRedMask);
	}
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

cv::Rect Perception::maxBoundingBox(const cv::Rect& largestBoundingBox, const cv::Rect& currentBoundingBox) {
	if (currentBoundingBox.width * currentBoundingBox.height > largestBoundingBox.width * largestBoundingBox.height) {
		return (currentBoundingBox);
	}
	return (largestBoundingBox);
}

void Perception::setEnemyPixels(const cv::Rect& largestBoundingBox, cv::Mat& rgbRedMask) {
	int x = largestBoundingBox.x;
	int y = largestBoundingBox.y;
	int w = largestBoundingBox.width;
	int h = largestBoundingBox.height;

	_enemyPixels.clear();
	for (int i = x; i < x + w; ++i) {
		for (int j = y; j < y + h; ++j) {
			if (rgbRedMask.at<uchar>(j, i) == 255) {
				_enemyPixels.push_back({i, j});
			}
		}
	}
}

void Perception::setEnemyBearing(const cv::Rect boundingBox) {
	int		cameraWidth = _camera.getWidth();
	double	boxCenterX = boundingBox.x + boundingBox.width / 2.;
	double	screenDistance = boxCenterX - cameraWidth / 2.;
	_enemyBearing = (screenDistance / cameraWidth) * _camera.getHorizontalFov();
	_enemyBearing = -_enemyBearing;
}

double Perception::calculateEnemyRange() {
	auto begin = _enemyPixels.begin();
	auto end = _enemyPixels.end();

	_enemyRange	= LIDAR_MAX_RANGE;
	for (auto& point: _lidar.getVisiblePoints()) {
		if (std::find(begin, end, point.imagePosition) != end) {
			_enemyRange = std::min(_enemyRange, point.getDistance());
		}
	}
	return _enemyRange;
}

void Perception::drawLidarPointsInImage() {
	_image.at<cv::Vec3b>(0, 0) = cv::Vec3b (0, 0, 255);
	_image.at<cv::Vec3b>(1, 0) = cv::Vec3b (0, 0, 255);
	_image.at<cv::Vec3b>(0, 1) = cv::Vec3b (0, 0, 255);
	_image.at<cv::Vec3b>(1, 1) = cv::Vec3b (0, 0, 255);
	for (const auto& point: _lidar.getVisiblePoints()) {
//		std::cout << "x: " << point.imagePosition.x << " y: " << point.imagePosition.y << std::endl;
		_image.at<cv::Vec3b>(point.imagePosition.y, point.imagePosition.x) = cv::Vec3b(255, 255, 255);
	}
}
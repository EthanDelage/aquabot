#include "Perception.hpp"

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
		_enemyFound = true;
		setEnemyPixels(largestBoundingRect, rgbRedMask);
		setEnemyBearing(largestBoundingRect);
	}
	else {
		_enemyFound = false;
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

	_enemyRangeMin = LIDAR_MAX_RANGE;
	_enemyRangeMax = -1;
	for (auto& point: _lidar.getVisiblePoints()) {
		if (std::find(begin, end, point.imagePosition) != end) {
			double distance = point.getDistance();
			_enemyRangeMin = std::min(_enemyRangeMin, distance);
			_enemyRangeMax = std::max(_enemyRangeMax, distance);
		}
	}
	return _enemyRangeMin;
}

void Perception::drawLidarPointsInImage() {
	for (const auto& point: _lidar.getVisiblePoints()) {
		_image.at<cv::Vec3b>(point.imagePosition.y, point.imagePosition.x) = cv::Vec3b(255, 255, 255);
	}
}

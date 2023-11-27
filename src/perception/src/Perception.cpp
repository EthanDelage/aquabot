#include <cv_bridge/cv_bridge.h>

#include "Perception.hpp"

using std::placeholders::_1;

Perception::Perception(): Node("perception") {
	_imageSubscriber = create_subscription<sensor_msgs::msg::Image>("/wamv/sensors/cameras/main_camera_sensor/image_raw",
			10, std::bind(&Perception::imageCallback, this, _1));
}

void Perception::imageCallback(sensor_msgs::msg::Image::SharedPtr msg) {
	std::cout << "Image callback" << std::endl;

	try {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		_image = cv_ptr->image;

		cv::imshow("Image", _image); // Afficher l'image dans la fenêtre

		cv::waitKey(1);
	} catch (std::exception const & e){
		RCLCPP_ERROR(this->get_logger(), "imageCallback() exception: %s", e.what());
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

	cv::Rect*	largestBoundingBox = nullptr;

	for (const auto& contour : redContours) {
	    if (isEnemyContour(contour, greenContours)) {
	        cv::Rect boundingRect = cv::boundingRect(contour);

	        largestBoundingBox = maxBoundingBox(largestBoundingBox, &boundingRect);

	        cv::rectangle(_image, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(0, 255, 0), 1);
	    }
	}

	if (largestBoundingBox != nullptr) {
	    int x = largestBoundingBox->x;
	    int y = largestBoundingBox->y;
	    int w = largestBoundingBox->w;
	    int h = largestBoundingBox->h;

	    redPixels.clear();

	    for (int i = x; i < x + w; ++i) {
	        for (int j = y; j < y + h; ++j) {
	            if (rgbRedMask.at<uchar>(j, i) == 255) {
	                redPixels.push_back(std::make_pair(i, j));
	            }
	        }
	    }
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

	return x2 <= x1 && x1 + w1 <= x2 + w2 && y2 <= y1 && y1 + h1 <= y2 + h2;
}

cv::Rect* Perception::maxBoundingBox(cv::Rect const * largestBoundingBox, cv::Rect const * currentBoundingBox) {
	if (largestBoundingBox == nullptr || currentBoundingBox->w * currentBoundingBox->h > largestBoundingBox->w * largestBoundingBox->h) {
		return (currentBoundingBox);
	}
	return (largestBoundingBox);
}
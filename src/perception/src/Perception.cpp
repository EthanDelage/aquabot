#include <memory>
#include <numeric>
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
	_imuSubscriber = create_subscription<sensor_msgs::msg::Imu>(
			"/wamv/sensors/imu/imu/data",
			10,
			std::bind(&Perception::imuCallback, this, _1));
	_gpsSubscriber = create_subscription<sensor_msgs::msg::NavSatFix>(
		"/wamv/sensors/gps/gps/fix",
		10,
		std::bind(&Perception::gpsCallback, this, _1));
	_perceptionPublisher = create_publisher<ros_gz_interfaces::msg::ParamVec>(
			"/perception/pinger",
			10);
	_alertPublisher = create_publisher<geometry_msgs::msg::PoseStamped>(
			"/vrx/patrolandfollow/alert_position",
			10);
	_enemyFound = false;
	_imageReceived = false;
	_cameraReceived = false;
	_imuPing = false;
	_gpsPing = false;
	_enemyRangeMin = LIDAR_MAX_RANGE;
}

void Perception::imageCallback(sensor_msgs::msg::Image::SharedPtr msg) {
	if (_lidarProcess) {
		return;
	}
	try {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		_image = cv_ptr->image;
		detectRedBoat();
		if (_enemyFound && _gpsPing && _imuPing) {
			publishPathfinding();
		}
		_imageReceived = true;
	} catch (const std::exception& e){
		RCLCPP_ERROR(this->get_logger(), "imageCallback() exception: %s", e.what());
	}
}

void Perception::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
	auto start = std::chrono::high_resolution_clock::now();
	if (_imageReceived && _cameraReceived) {
		_lidarProcess = true;
		_lidar.parsePoints(msg);
		_lidar.setVisiblePoints(_camera);
		calculateEnemyRange();
		_lidarProcess = false;
		drawLidarPointsInImage();
		cv::imshow("Image", _image);
		cv::waitKey(1);
	}
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Time in ms: " << duration.count() << std::endl;
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

void Perception::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
	calculateYaw(msg->orientation);
	if (_gpsPing && _enemyFound) {
		calculateEnemyPos();
		publishAlert();
	}
	_imuPing = true;
}

void Perception::gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	double latitude = msg->latitude;
	double longitude = msg->longitude;
	calculateMapPos(latitude, longitude);
	_gpsPing = true;
}


void Perception::publishPathfinding() {
	auto	paramVecMsg = ros_gz_interfaces::msg::ParamVec();
	rcl_interfaces::msg::Parameter	rangeMsg;
	rcl_interfaces::msg::Parameter	bearingMsg;
	rcl_interfaces::msg::Parameter	desiredRangeMsg;
	rcl_interfaces::msg::Parameter	xMapPos;
	rcl_interfaces::msg::Parameter	yMapPos;

	rangeMsg.name = "range";
	rangeMsg.value.double_value = _enemyRangeMin;
	paramVecMsg.params.push_back(rangeMsg);

	bearingMsg.name = "bearing";
	bearingMsg.value.double_value = _enemyBearing;
	paramVecMsg.params.push_back(bearingMsg);

	desiredRangeMsg.name = "desiredRange";
	desiredRangeMsg.value.double_value = DESIRED_RANGE;
	paramVecMsg.params.push_back(desiredRangeMsg);

	xMapPos.name = "x";
	xMapPos.value.double_value = _enemyMapPos[0];
	paramVecMsg.params.push_back(xMapPos);

	yMapPos.name = "y";
	yMapPos.value.double_value = _enemyMapPos[1];
	paramVecMsg.params.push_back(yMapPos);
	_perceptionPublisher->publish(paramVecMsg);
}


void Perception::publishAlert() {
	if (_enemyFound && _enemyRangeMin >= 130)
		return;
	if (_rangeHistory.size() != 0) {
		double average = std::accumulate(_rangeHistory.begin(),
										 _rangeHistory.end(), 0.0)
												 / _rangeHistory.size();
		if (std::abs(average - _enemyRangeMin) > ALLOW_ALERT_ERROR_THRESHOLD)
			return;
	}
	_rangeHistory.push_back(_enemyRangeMin);
	if (_rangeHistory.size() > 10) {
		_rangeHistory.erase(_rangeHistory.begin());
	}
	auto poseStamped = geometry_msgs::msg::PoseStamped();
	poseStamped.pose.position.x = _enemyGPSPos[1];
	poseStamped.pose.position.y = _enemyGPSPos[0];
	_alertPublisher->publish(poseStamped);
}

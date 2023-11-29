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
	_imuSubscriber = create_subscription<sensor_msgs::msg::Imu>(
			"/wamv/sensors/imu/imu/data",
			10,
			std::bind(&Perception::imuCallback, this, _1));
	_gpsSubscriber = create_subscription<sensor_msgs::msg::NavSatFix>(
		"/wamv/sensors/gps/gps/fix",
		10,
		std::bind(&Perception::gpsCallback, this _1));
	_navigationPublisher = create_publisher<ros_gz_interfaces::msg::ParamVec>(
			"/perception/pinger", 10);
	_enemyFound = false;
	_imageReceived = false;
	_cameraReceived = false;
	_enemyRange = LIDAR_MAX_RANGE;
}

void Perception::imageCallback(sensor_msgs::msg::Image::SharedPtr msg) {
	try {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		_image = cv_ptr->image;
		detectRedBoat();
		if (_enemyFound) {
			publishNavigation();
		}
		_imageReceived = true;
	} catch (const std::exception& e){
		RCLCPP_ERROR(this->get_logger(), "imageCallback() exception: %s", e.what());
	}
}

void Perception::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
	if (_imageReceived && _cameraReceived) {
		_lidar.parsePoints(msg);
		auto start = std::chrono::high_resolution_clock::now();
		_lidar.setVisiblePoints(_camera);
		auto end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		std::cout << "Time in ms: " << duration.count() << std::endl;
		calculateEnemyRange();
		drawLidarPointsInImage();
		cv::imshow("Image", _image);
		cv::waitKey(1);
	}
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
	calculateYaw(msg->orient);
}

void Perception::calculateYaw(const geometry_msgs::msg::Quaternion& orientation) {
	double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
	double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
	_boatOrientation = std::atan2(siny_cosp, cosy_cosp);
}

void Perception::calculateEnemyPos() {
	double buoyOrientation = convertToMinusPiPi(_orientation + _enemyBearing);
	_buoyPos.x = _buoyRange * std::cos(buoyOrientation) + _boatPos.x;
	_buoyPos.y = _buoyRange * std::sin(buoyOrientation) + _boatPos.y;
}

double Perception::convertToMinusPiPi(double angleRadians) {
	while (angleRadians <= -M_PI) {
		angleRadians += 2.0 * M_PI;
	}
	while (angleRadians > M_PI) {
		angleRadians -= 2.0 * M_PI;
	}
	return (angleRadians);
}

void Perception::publishNavigation() {
	auto	paramVecMsg = ros_gz_interfaces::msg::ParamVec();
	rcl_interfaces::msg::Parameter	rangeMsg;
	rcl_interfaces::msg::Parameter	bearingMsg;
	rcl_interfaces::msg::Parameter	desiredRangeMsg;

	rangeMsg.name = "range";
	rangeMsg.value.double_value = _enemyRange;
	bearingMsg.name = "bearing";
	bearingMsg.value.double_value = _enemyBearing;
	desiredRangeMsg.name = "desiredRange";
	desiredRangeMsg.value.double_value = DESIRED_RANGE;

	paramVecMsg.params.push_back(rangeMsg);
	paramVecMsg.params.push_back(bearingMsg);
	paramVecMsg.params.push_back(desiredRangeMsg);
	_navigationPublisher->publish(paramVecMsg);
}

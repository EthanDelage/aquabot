#include "Pathfinding.hpp"

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"

void Pathfinding::pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg) {
	if (_state >= FOLLOW_STATE)
		return;
	for (const auto &param: msg->params) {
		std::string name = param.name;

		if (name == "bearing")
			_targetBearing = param.value.double_value;
		else if (name == "range")
			_targetRange = param.value.double_value;
	}
	_buoyPing = true;
}

void Pathfinding::perceptionCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg) {
	if (_state < FOLLOW_STATE) {
		if (!_enemyPing)
			checkEnemyCollision(msg);
		return;
	}
	int scanValue = 0;
	for (const auto &param: msg->params) {
		std::string name = param.name;

		if (name == "bearing")
			_targetBearing = param.value.double_value;
		else if (name == "range")
			_targetRange = param.value.double_value;
		else if (name == "desiredRange")
			_targetDesiredRange = param.value.double_value;
		else if (name == "x")
			_target.position.x = param.value.double_value;
		else if (name == "y")
			_target.position.y = param.value.double_value;
		else if (name == "scan")
			_scan = param.value.bool_value;
		else if (name == "scanOrientation")
			scanValue = param.value.integer_value;
	}
	if (_scan) {
		publishScan(scanValue);
		return;
	}
	_path = calculatePath(_boatPos);
}

void Pathfinding::taskInfoCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg) {
	for (const auto & param: msg->params) {
		if (param.name == "state" && param.value.string_value == "finished") {
			rclcpp::shutdown();
			exit(0);
		}
	}
}

void Pathfinding::publishScan(double value) {
	auto	paramVecMsg = ros_gz_interfaces::msg::ParamVec();
	rcl_interfaces::msg::Parameter	scanMsg;
	rcl_interfaces::msg::Parameter	scanValue;

	scanMsg.name = "scan";
	scanMsg.value.bool_value = true;
	scanValue.name = "scanValue";
	scanValue.value.double_value = value;
	paramVecMsg.params.push_back(scanMsg);
	paramVecMsg.params.push_back(scanValue);

	_publisherRangeBearing->publish(paramVecMsg);
}

void Pathfinding::phaseCallback(std_msgs::msg::UInt32::SharedPtr msg) {
	_state = msg->data;
}

void Pathfinding::gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	double latitude = msg->latitude;
	double longitude = msg->longitude;

	_boatPos = calculateMapPos(latitude, longitude);
	_gpsPing = true;
	if (_buoyPosCalculate && !_pathCalculated) {
		_path = calculatePath(_boatPos);
		_pathCalculated = true;
	}
}

void Pathfinding::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
	if (_scan && _state >= 2)
		return;
	_orientation = calculateYaw(msg->orientation);
	_imuPing = true;
	if (_buoyPing && _gpsPing && !_buoyPosCalculate)
		addBuoy();
	if (_gpsPing && _pathCalculated && !_path.empty()) {
		if (_path[0].x == _target.position.x && _path[0].y == _target.position.y) {
			publishRangeBearing(std::pair<double, double>(_targetRange, _targetBearing), _targetDesiredRange);
		}
		else
			publishRangeBearing(calculateRangeBearing(), MAX_CHECKPOINT_RANGE);
	}
}

void Pathfinding::alliesCallback(geometry_msgs::msg::PoseArray::SharedPtr msg) {
	std::vector<std::pair<point_t, double>>	closeAllies;

	for (const auto &pose : msg->poses)
	{
		std::pair<point_t, double>	allyInfo;
		allyInfo.first.x = pose.position.x;
		allyInfo.first.y = pose.position.y;
		allyInfo.first = calculateMapPos(allyInfo.first.x, allyInfo.first.y);
		allyInfo.second = calculateYaw(pose.orientation);
		if (calculateDist(_boatPos, allyInfo.first) > MIN_ALLY_RANGE)
			continue;
		closeAllies.push_back(allyInfo);
	}
	if (!closeAllies.empty()) {
		_path = calculatePathWithAllies(_boatPos, closeAllies);
	}
}
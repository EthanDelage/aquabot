#include "Navigation.hpp"

using std::placeholders::_1;

Navigation::Navigation() : Node("navigation") {
	_pinger = create_subscription<ros_gz_interfaces::msg::ParamVec>("/wamv/sensors/acoustics/receiver/range_bearing", 10,
				std::bind(&Navigation::pingerCallback, this, _1));

}

void Navigation::pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg) {
	for (const auto &param: msg->params) {
		std::string name = param.name;

		if (name == "bearing")
			_bearing = param.value.double_value;
		else if (name == "range")
			_range = param.value.double_value;
	}
}
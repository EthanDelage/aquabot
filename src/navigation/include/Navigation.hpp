#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

# include "rclcpp/rclcpp.hpp"
# include "ros_gz_interfaces/msg/param_vec.hpp"

class Navigation : public rclcpp::Node {

public:
	Navigation();

private:
	rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr	_pinger;

	double	_bearing;
	double	_range;

	void	pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg);

};


#endif

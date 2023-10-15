#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

# include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/float64.hpp"
# include "ros_gz_interfaces/msg/param_vec.hpp"
# include <cmath>

# define THRUST_MAX 	12000
# define NAV_THRUST_MIN	1000
# define POS_MAX		M_PI_4
# define POS_MIN		-POS_MAX

class Navigation : public rclcpp::Node {

public:
	Navigation();

private:
	double	_bearing;
	double	_range;

	rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr	_pinger;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr				_publisherThrust;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr				_publisherPos;


	void	pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg);
	void	setHeading(double bearing, double range);
	double	calculateThrust(double regulation);
	double	regulator(double bearing);

};


#endif

#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

# include "point.hpp"

# include <cmath>

# include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/float64.hpp"
# include "ros_gz_interfaces/msg/param_vec.hpp"
# include "sensor_msgs/msg/nav_sat_fix.hpp"
# include "tf2_ros/buffer.h"
# include "tf2_ros/transform_listener.h"
# include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

# define THRUST_MAX 	12000
# define NAV_THRUST_MIN	1000
# define POS_MAX		M_PI_4
# define POS_MIN		(-POS_MAX)
# define MAX_BUOY_RANGE	20

class Navigation : public rclcpp::Node {

public:
	Navigation();
	Navigation(double gain, double sigma);

private:
	double							_bearing;
	double							_range;
	double							_desiredRange;
	double							_gain;
	double							_sigma;
	bool							_benchmark;

	rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr	_pinger;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr		_alliesPos;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr				_publisherThrust;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr				_publisherPos;


	void	pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg);
	void 	alliesPosCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void	setHeading(double bearing, double range);
	double	calculateThrust(double regulation);
	double	regulator(double bearing);
};


#endif

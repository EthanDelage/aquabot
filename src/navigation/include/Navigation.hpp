#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

# include "point.hpp"
# include "Pathfinding.hpp"
# include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/float64.hpp"
# include "ros_gz_interfaces/msg/param_vec.hpp"
# include "sensor_msgs/msg/nav_sat_fix.hpp"
# include "sensor_msgs/msg/imu.hpp"
# include "geometry_msgs/msg/quaternion.hpp"
# include "geometry_msgs/msg/quaternion_stamped.hpp"
# include <cmath>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

# define THRUST_MAX 	12000
# define NAV_THRUST_MIN	1000
# define POS_MAX		M_PI_4
# define POS_MIN		(-POS_MAX)
# define MAX_BUOY_RANGE	20
# define LONGITUDE_0	(-4.980343472857843)
# define LATITUDE_0		48.043601874279716
# define LONGITUDE_1	(-4.9722961068569775)
# define LATITUDE_1		48.04899798353722



class Navigation : public rclcpp::Node {

public:
	Navigation();
	Navigation(double gain, double sigma);

private:
	bool							_buoyPing;
	bool 							_gpsPing;
	double							_bearing;
	double							_range;
	point_t							_buoyPos;
	point_t							_boatPos;
	double							_gain;
	// Écart-type
	double							_sigma;
	bool							_benchmark;
	Pathfinding						*_pathfinding;
	geometry_msgs::msg::Quaternion	_orientation;

	rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr	_pinger;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr		_gps;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr				_imu;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr		_alliesPos;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr				_publisherThrust;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr				_publisherPos;


	void	pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg);
	void 	gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void 	imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);
	void 	alliesPosCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void	setHeading(double bearing, double range);
	double	calculateThrust(double regulation);
	double	regulator(double bearing);
	double	convertToMinusPiPi(double angleRadians);
	void	calculateMapPos(double latitude, double longitude);
};


#endif

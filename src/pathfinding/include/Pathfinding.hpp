#ifndef PATHFINDING_HPP
# define PATHFINDING_HPP

# include <list>

# include "Graph.hpp"
# include "point.hpp"
# include "rclcpp/rclcpp.hpp"
# include "sensor_msgs/msg/imu.hpp"
# include "std_msgs/msg/float64.hpp"
# include "geometry_msgs/msg/pose.hpp"
# include "sensor_msgs/msg/nav_sat_fix.hpp"
# include "geometry_msgs/msg/pose_array.hpp"
# include "ros_gz_interfaces/msg/param_vec.hpp"

# define OBSTACLE_FILE			"src/pathfinding/obstacles.txt"
# define MAX_CHECKPOINT_RANGE	11
# define MAX_BUOY_RANGE			20
# define LONGITUDE_0			(-4.980343472857843)
# define LATITUDE_0				48.043601874279716
# define LONGITUDE_1			(-4.9722961068569775)
# define LATITUDE_1				48.04899798353722
# define MIN_ALLY_RANGE			100

typedef struct rectangle_s {
	size_t	id;
	point_t point[4];
} rectangle_t;

typedef struct checkpoint_s {
	size_t	graphIndex;
	point_t position;
} checkpoint_t;

class Pathfinding : public rclcpp::Node {

private:
	// Cartography attributes
	std::vector<rectangle_t>				_obstacles;
	std::vector<checkpoint_t>				_checkpoints;
	Graph									_obstaclesGraph;

	// Buoy attributes
	checkpoint_t				_buoy;
	bool						_buoyPing;
	double						_buoyRange;
	double						_buoyBearing;
	bool 						_buoyPosCalculate;

	// Boat attributes
	point_t						_boatPos;
	double						_orientation;
	bool						_imuPing;
	bool						_gpsPing;
	bool 						_pathCalculated;
	std::vector<point_t>		_path;

	// Publisher/Subscriber
	rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr	_pinger;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr		_gps;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr				_imu;
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr		_allies;
	rclcpp::Publisher<ros_gz_interfaces::msg::ParamVec>::SharedPtr		_publisherRangeBearing;

	// Callback functions
	void	pingerCallback(ros_gz_interfaces::msg::ParamVec::SharedPtr msg);
	void 	gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
	void 	imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);
	void	alliesCallback(geometry_msgs::msg::PoseArray::SharedPtr msg);

	// Parsing functions
	int 				parseObstacles();
	static rectangle_t	parseBoundingBox(std::string const & strBoundingBox);

	// Djikstra algorithm functions
	std::vector<std::pair<size_t, double>>	djikstra(size_t start, size_t end, Graph const & graph);
	std::vector<point_t>					convertNodeToPoint(std::list<size_t> nodePath);
	std::vector<point_t>					convertDjikstraToPoint(std::vector<std::pair<size_t, double>> reversePath, size_t start, size_t end);
	static bool								isVisited(size_t index, std::vector<size_t> const & visited);
	static std::pair<size_t, double>		getMinNode(std::vector<std::pair<size_t, double>> const & path, std::vector<size_t> const & visited);
	static double							calculateDist(point_t a, point_t b);

	// Collision functions
	bool 			isHitObstacle(point_t const start, point_t const end, rectangle_t const & lhs, rectangle_t const & rhs);
	bool 			isHitObstacle(point_t const start, point_t const end, rectangle_t const & dest);
	bool 			isHitObstacle(point_t const start, point_t const end);
	static bool		isHitItself(point_t const start, point_t const end, rectangle_t const & obstacle, size_t index);
	static bool		isIntersect(point_t a1, point_t a2, point_t b1, point_t b2);
	static int 		orientation(point_t p, point_t q, point_t r);
	static bool		onSegment(point_t p, point_t q, point_t r);
	static bool		areRectangleEqual(rectangle_t const & lhs, rectangle_t const & rhs);

	// Graph generation functions
	void	generateObstaclesGraph();
	void 	addObstacleAdjList(rectangle_t const & obstacle, size_t index);
	void	generateAdjList(rectangle_t const & lhs, rectangle_t const & rhs);
	void	generateCheckpointAdjList(checkpoint_t checkpoint, Graph& graph);

	// Buoy functions
	void			calculateBuoyPos();
	static double	convertToMinusPiPi(double angleRadians);

	// Boat functions
	point_t						calculateMapPos(double latitude, double longitude);
	double						calculateYaw(geometry_msgs::msg::Quaternion const & orientation);
	std::vector<point_t>		calculatePath(point_t boatPos);
	std::vector<point_t>		calculatePathWithAllies(point_t boatPos, std::vector<std::pair<point_t, double>> allies);
	std::pair<double, double>	calculateRangeBearing();
	void						publishRangeBearing(std::pair<double, double> const & rangeBearing, double desiredRange);
	rectangle_t					calculateAllyBoundingBox(std::pair<point_t, double> const & ally, size_t id);

public:
	Pathfinding();

	int						init();
	void					addBuoy();
	size_t					addCheckPoint(point_t checkpointPos, Graph& graph);

};

#endif

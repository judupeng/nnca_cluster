
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>

#include "nnca_cluster/struct_define.h"

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <tf/tf.h>

//#include <nnca_cluster/utils.h>


using namespace std;

class MemoryDecision{

private:
	std::deque<geometry_msgs::PoseStamped> walked_waypoints;

	const int WAYPOINT_CNT = 100;

	const int P_DISTANCE = 2;//2M

	const int SEARCH_R = 1;

	bool approx_points(geometry_msgs::PoseStamped a,geometry_msgs::PoseStamped b);

	int get_include_waypoint_cnt(double current_angle,double turn_angle,std::vector<double> waypoint_yaw_set);

	Obstacle get_choose_yaw(geometry_msgs::PoseStamped current_pose,std::vector<Obstacle> able_walk_yaw);

	void get_waypoint_current_angle(geometry_msgs::PoseStamped current_pose,std::vector<double> *waypoint_yaw_set);

	double get_distance(geometry_msgs::PoseStamped a,geometry_msgs::PoseStamped b);

	double get_pose_from_xyzw(geometry_msgs::PoseStamped current_pose);
public:
	MemoryDecision();
	~MemoryDecision();

	Obstacle choose_walk_yaw(geometry_msgs::PoseStamped current_pose,std::vector<Obstacle> able_walk_yaw);

	void save_walked_waypoint(geometry_msgs::PoseStamped current_status_pose);

	bool has_walked(geometry_msgs::PoseStamped current_pose);
};

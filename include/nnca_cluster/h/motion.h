#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
using namespace std;


double normalize_angle(double angle);

void  get_odom(geometry_msgs::Point &position_tmp, double &yaw_tmp,bool &is_get);

void  turn_goal_angle(double goal_angle,double angular_speed,geometry_msgs::Twist &twist_cmd,
        ros::Publisher &cmd_vel_pub_);

void  go_straight(double goal_distance,double linear_speed,geometry_msgs::Twist &twist_cmd,
        ros::Publisher &cmd_vel_pub);

void  go_curve(double goal_distance,double linear_speed,double goal_angle,double angle_bias,geometry_msgs::Twist &twist_cmd,
        ros::Publisher &cmd_vel_pub);

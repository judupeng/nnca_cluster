#include "nnca_cluster/motion.h"


double normalize_angle(double angle)
{
    while (angle > 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
    return angle;
}

void  get_odom(geometry_msgs::Point &position_tmp, double &yaw_tmp,bool &is_get)
{
    is_get=0;
    tf::TransformListener listener;
    tf::StampedTransform transform_;
    try{
      listener.lookupTransform("/odom","/base_footprint",ros::Time(0), transform_);
      position_tmp.x=transform_.getOrigin().x();
      position_tmp.y=transform_.getOrigin().y();
      position_tmp.z=transform_.getOrigin().z();
      yaw_tmp=tf::getYaw(transform_.getRotation())*180/M_PI;
      is_get=1;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("2323 %s ",ex.what());
      is_get=0;
    }
}

void  turn_goal_angle(double goal_angle,double angular_speed,geometry_msgs::Twist &twist_cmd,
        ros::Publisher &cmd_vel_pub)
{
    geometry_msgs::Point position;
    double turn_angle = 0;
    double last_angle =0;
    double delta_angle=0;
    double angular_tolerance=2.5;
    double yaw=0;
    bool  is_get=0;
    get_odom(position, yaw, is_get);
    last_angle=yaw;
    twist_cmd.linear.x = 0;
    twist_cmd.angular.z = angular_speed;
    if (is_get==1)
    {
        while (abs(turn_angle + angular_tolerance) < abs(goal_angle))
        {
            cmd_vel_pub.publish(twist_cmd);
            ros::Duration(0.1).sleep();
            get_odom(position, yaw, is_get);
            if (is_get==1)
            {
                delta_angle = normalize_angle(yaw - last_angle);
                turn_angle += delta_angle;
                last_angle = yaw;
            }
            else 
                cmd_vel_pub.publish(geometry_msgs::Twist());
        }
    }
    twist_cmd=geometry_msgs::Twist();
}

void  go_straight(double goal_distance,double linear_speed,geometry_msgs::Twist &twist_cmd,
        ros::Publisher &cmd_vel_pub)
{
    geometry_msgs::Point position;
    double distance_tolerance=0.1;
    double distance = 0;
    double x_start = 0;
    double y_start = 0;
    double yaw = 0;
    bool is_get=0;
    get_odom(position, yaw, is_get);
    x_start = position.x;
    y_start = position.y;
    twist_cmd.linear.x = linear_speed;
    twist_cmd.angular.z =0;
    while (distance < goal_distance)
    {
        cmd_vel_pub.publish(twist_cmd);
        ros::Duration(0.1).sleep();
        get_odom(position, yaw, is_get);
        if (is_get==1)
            distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2));
        else 
            cmd_vel_pub.publish(geometry_msgs::Twist());
    }
}

void  go_curve(double goal_distance,double linear_speed,double goal_angle,double angle_bias,geometry_msgs::Twist &twist_cmd,
        ros::Publisher &cmd_vel_pub)
{
    geometry_msgs::Point position;
    double distance_tolerance=0.1;
    double distance = 0;
    double x_start = 0;
    double y_start = 0;
    double turn_angle = 0;
    double last_angle =0;
    double delta_angle=0;
    double angular_tolerance=2.5;
    double yaw = 0;
    bool is_get=0;
    get_odom(position, yaw, is_get);
    x_start = position.x;
    y_start = position.y;
    twist_cmd.linear.x = linear_speed;
    twist_cmd.angular.z =  angle_bias*linear_speed/goal_distance/45;   //当V固定时，w=V*d_goal/d_goal/45;
    while (distance < goal_distance && abs(turn_angle + angular_tolerance) < abs(goal_angle))
    {
        cmd_vel_pub.publish(twist_cmd);
        ros::Duration(0.1).sleep();
        get_odom(position, yaw, is_get);
        if (is_get==1)
        {
            distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2));
            delta_angle = normalize_angle(yaw - last_angle);
            turn_angle += delta_angle;
            last_angle = yaw;
        }
        else 
            cmd_vel_pub.publish(geometry_msgs::Twist());
    }
}
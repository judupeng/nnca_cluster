#ifndef _UTILS_H_
#define _UTILS_H_


#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "nnca_cluster/lla_xyz.h"

//获取路线距离当前位置最近点的索引
int get_closest_distance(nav_msgs::Path &route,geometry_msgs::PoseStamped *current_gps_pose);

//过滤掉最近点之前的路线点
void pro_closest_route(nav_msgs::Path &route,geometry_msgs::PoseStamped *current_gps_pose);

//get line equation

Line get_line(Point3 a,Point3 b);

void get_cross(Line l1,Line l2,Point3 *p);

Point3 get_closest_point(Point3 p,Point3 a,Point3 b);

Point3 find_closest_ponit3(Point3 current_point,std::vector<Point3> route);

bool route_ready(Point3 current_point,std::vector<Point3> route);

#endif
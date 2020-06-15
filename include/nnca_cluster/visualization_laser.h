# ifndef __VISUALIZATION_LASER__
# define __VISUALIZATION_LASER__

#include <ros/ros.h>
#include "nnca_cluster/struct_define.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

void showArrowGoal(visualization_msgs::Marker &arrow_goal,Obstacle &obstacle_temp,const sensor_msgs::LaserScan& msg,Laser_info laser_info);
void showArrowAead(visualization_msgs::Marker &arrow_head,const sensor_msgs::LaserScan& msg,double lock_angle,int n);

void showMergerLaser(const sensor_msgs::LaserScan& msg, ros::Publisher &scan_debug_marker_pub);

void showPointsCluster(Obstacle &obstacle_temp,int class_id,const sensor_msgs::LaserScan& msg,
    visualization_msgs::Marker &line_list,visualization_msgs::Marker &text_view,Laser_info   laser_info);

void showPointsPlusterOrbbec(Obstacle &obstacle_temp,const sensor_msgs::LaserScan& msg,
    visualization_msgs::Marker &line_list_orbbec,visualization_msgs::Marker &text_view_orbbec,Laser_info   laser_info);

void showDepthPointsProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_Y_cloud_ptr,const sensor_msgs::PointCloud2ConstPtr& msg,
ros::Publisher &depth_projection_markers_pub);


#endif
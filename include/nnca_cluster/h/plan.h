#ifndef  __PLAN_H__
#define  __PLAN_H__

#include "nnca_cluster/struct_define.h"
#include "nnca_cluster/utils.h"
#include "nnca_cluster/priority_cal.h"

#include <ros/ros.h>

class Plan
{
    private:
    list<int>  m_list_sign_cun;
    list<int>  m_list_sign_cun_orbbec;
    Obstacle   m_last_sel_Obstacle;
    Plan_info  m_plan_info;
    Inflate_info inflate_info;
    Laser_info  laser_info;

    nav_msgs::Path m_plan_path_on_map_dynamic;
    nav_msgs::Path m_plan_path_on_map;
    geometry_msgs::PoseStamped m_current_gps_pose;


    Plan_State PLAN_STATE;
    Move_State MOVE_STATE;
    Sensor_mode SENSOR_MODE;
    Twist_def  twist_def;
    double m_global_direction;
    int   turning_angle;
    int   turning_angle_orbbec;
    int   n_sel_max_pri_id;
    int   n_sign_cunt;
    int   n_sign_cunt_orbbec;
    int   n_sign_sum;
    int   COUNT;
    public:
    Plan(Plan_info plan_info,nav_msgs::Path &plan_path_on_map,geometry_msgs::PoseStamped current_gps_pose, 
    Car_info  car_info,Laser_info  laser_info_tmp);
    bool  inflate_detect(const sensor_msgs::LaserScan& msg,sensor_msgs::LaserScan &inflat_msg_left,
                    sensor_msgs::LaserScan  &inflat_msg_right, Inflate_info &inflate_info);
    void update_sign(vector<Obstacle>& v_obstacle_tmp);
    void find_max_pri(vector<Obstacle>& v_obstacle_tmp);
    void find_lock_max_pri(vector<Obstacle>& v_obstacle_tmp);

    void plan_init(vector<Obstacle>& v_obstacle_tmp);

    void find_max_pri_gpsmode(vector<Obstacle>& v_obstacle_tmp,int lock_mode);
    void set_unfreeze_move_stete(vector<Obstacle>& v_obstacle_tmp,sensor_msgs::LaserScan &scan_msg);
    void set_freeze_move_stete(vector<Obstacle>& v_obstacle_tmp,sensor_msgs::LaserScan &scan_msg);
    void unfreeze_plan(vector<Obstacle>& v_obstacle_tmp,sensor_msgs::LaserScan &scan_msg,ros::Publisher &plan_path_pub);
    void freeze_plan(vector<Obstacle>& v_obstacle_tmp,sensor_msgs::LaserScan &scan_msg,ros::Publisher &plan_path_pub);


};

#endif
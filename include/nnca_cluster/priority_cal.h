#include "nnca_cluster/struct_define.h"
#include "nnca_cluster/utils.h"

using namespace std;

void priorityQueuePri(int (&sel_max_n_pri_id)[5],int n,vector<Obstacle>& v_obstacle_tmp);

void priorityQueuePriLockAngle(int (&sel_max_n_pri_id)[5],int n,double lock_angle,vector<Obstacle>& v_obstacle_tmp);

void pri_Set(Obstacle &obstacle_temp,double dmin,Car_info car_info);   //有反射数据的优先级

 
void pri_Set_MAX_RANGE(Obstacle &obstacle_temp,std::vector<double > threshold_arr);   //无反射数据的优先级

bool  inflate_detect(const sensor_msgs::LaserScan& msg,sensor_msgs::LaserScan &inflat_msg_left,
        sensor_msgs::LaserScan  &inflat_msg_right, Inflate_info &inflate_info,Laser_info laser_info);


void gpsMode(int lock_state,Obstacle_info & obstacle_info,SensorsInfo sensors_info,Path_map_info &path_map_info);  //lock_state=1 未锁定， 2 锁定
void  normalMode(int lock_state, Obstacle_info & obstacle_info);  //lock_state=1 未锁定， 2 锁定
int  signCount(Obstacle_info & obstacle_info);
void  normalModeOrbbec( Obstacle_info & obstacle_info);
double distance_Weigh(Obstacle &obstacle_temp);

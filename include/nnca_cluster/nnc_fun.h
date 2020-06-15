
#include "nnca_cluster/struct_define.h"


void pre_process( sensor_msgs::LaserScan& msg,Laser_info &laser_info);
double square_sum(double x,double x2,double y,double y2);
double rms_cal(double x,double x2,double y,double y2);
double cal_obstacle_din(int temp_start_id, int temp_end_id,Laser_info laser_info);
void calculating_statistics_head_tail(Laser_info laser_info,Obstacle &obstacle_head,Obstacle &obstacle_tail ,int select_);

void calculating_statistics_360(Laser_info laser_info, Obstacle &obstacle_temp);

bool check_new_class(SensorsInfo sensors_info,int indx_laser,int indx_angle,int select_);

void nnc_360_laser(vector<Obstacle>& v_obstacle_tmp,SensorsInfo sensors_info);
double cal_pria(Obstacle &obstacle_temp,int temp_start_id,int temp_end_id ,Laser_info laser_info);


void  nnc_fun(SensorsInfo sensors_info,vector<Obstacle>& v_obstacle_tmp);

void  nnc_fun_orbbec_laser(SensorsInfo sensors_info,const sensor_msgs::LaserScan& msg,vector<Obstacle>& v_obstacle_tmp);
void  nnc_xxx_laser(vector<Obstacle>& v_obstacle_tmp,const sensor_msgs::LaserScan& laser_tmp,SensorsInfo sensors_info);
double cal_obstacle_xxx_din(const sensor_msgs::LaserScan& laser_tmp,int temp_start_id, int temp_end_id,Laser_info laser_info);
void  calculating_statistics_xxx(Laser_info laser_info,const sensor_msgs::LaserScan& laser_tmp,Obstacle &obstacle_temp);


double cal_Angle(int laser_id, const sensor_msgs::LaserScan& laser_tmp);
double cal_Mid_Angle(Obstacle &obstacle, const sensor_msgs::LaserScan& laser_tmp);

void filter_mid_max3(sensor_msgs::LaserScan& msg,Laser_info laser_info,int _case=2);
void filter_mid3(sensor_msgs::LaserScan& msg);

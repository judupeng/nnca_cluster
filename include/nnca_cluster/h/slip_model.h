# ifndef _SLIP_MODEL_H_
# define _SLIP_MODEL_H_
#include "nnca_cluster/struct_define.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
class SlipModel
{
    public:
    nav_msgs::Odometry m_odomPre;
    deque<vector<double>> m_distanceArr;
    double m_deltDisArr[5];
    double m_disturbance;
    double m_deltaThetaImu;
    double m_deltaThetaOdom;
    double m_deltaTheta;
    double m_ThetaThreshold;
    double m_yawImuPre;
    double m_yawOdomPre;
    double m_deltaDistanceOdom;
    double m_deltaDistance;
    double m_wheel_interval;
    double m_delta_dR;
    double m_delta_dL;
    int  m_sgnCunt;
    int  m_rang_max_set;
    int  m_num;
    bool m_isSlip;
    bool m_imuFlag;

    public:
    SlipModel(double wheel_interval);
    void setDistanceArr(vector<double> distance_arr);
    int judgeSlipLaser(vector<double> distance_arr,int rang_max_set=4,double disturbance=0.01, int num=10);
    int cal();

    int judgeSlipOdomImu(sensor_msgs::Imu imu_msg,nav_msgs::Odometry &odom);
    int calOdomImu(nav_msgs::Odometry &odom);
    void correctModel_1(nav_msgs::Odometry &odom);
    void correctModel_2(nav_msgs::Odometry &odom);
    void calDeltXY(double &deltX,double &deltY );

};





# endif
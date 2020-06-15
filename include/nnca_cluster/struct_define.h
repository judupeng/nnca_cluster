#ifndef STRUCT_DEFINE_H_
#define STRUCT_DEFINE_H_

#include <sstream>
#include <cmath>
#include <numeric>
#include <list>
#include <vector>
#include <queue>
#include <string>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

    const double D_2_div_PI =2*M_PI;
    const double D_180_div_PI =180/M_PI;
    const double D_PI_div_2 =M_PI/2;
    const double D_PI_div_1_div_180 =M_PI*1.0f/180;

    const double COLOR_VALUE_MIN = 0.0;
    const double COLOR_VALUE_MAX = 1.0;
    const double COLOR_VALUE_MEDIAN = 0.5;
    const double COLOR_VALUE_LIGHT_LOW = 0.56;
    const double COLOR_VALUE_LIGHT_HIGH = 0.93;
 

    struct Obstacle{
            double dmax; //类最大距离
            double dmin;//类最小距离
            double dave;// 类与原点平均间距
            //double dcov;
            //double dd_d;
            double din2;// 类内间距的平方
            double angular; //类与当前方向夹角  范围是：-180~180度。 向左数开始0~180度， 向右数开始是-1度～-179度
            double linear_speed;
            double angular_speed;
            double pri;
            double pri_A;
            int LASRE_BEAM;
            int class_id;
            int start_id;
            int n_lasers;
            vector<int> laser_ids;
            bool operator < (const Obstacle & a) const
            {
                return pri<a.pri;
            }
        };


    struct Laser_info{

        sensor_msgs::LaserScan msg;
        double C_ranges_max;
        double range_min;
        double angle_increment;
        double C;
        double C_LASRE_THRESHOLD;
        int C_ranges_max_set;
        int LASRE_BEAM;
        int laser_begin_id;
    };

    struct Car_info{
        double max_vel_x;
        double min_vel_x;
        double max_turn_x;
        double acc_lim_x;
        double acc_lim_th;
        double  WIDTH;
        double  LONG;
        double  C_BODY;
        double  wheel_interval;
        double  wheel_diameter;
        double vel_factor;
        double turn_factor;
    };

    struct SensorsInfo
    {
        Car_info   car_info;
        Laser_info laser_info;
        Laser_info laser_info_orbbec;
    };

    struct Inflate_info
    {
        double inflate_l_r;
        double inflate_front;
        double inflate_back;
        int   left_top_lsaer_id;
        int   right_top_lsaer_id;
        int   left_lsaer_id;
        int   right_lsaer_id;
    }; 

    struct Plan_info
    {
        double lock_angle;
        double yaw_with_north;
        bool is_current_direction;
        bool is_global_direction;
    };


    struct Obstacle_info
    {
        vector<Obstacle> v_obstacle;
        Obstacle last_sel_Obstacle;
        list<int>  list_sign_cun;
        double lock_angle;
        double max_pri;
        double turning_angle;
        double real_distance;
        double real_angle;
        double left_distance;
        double mid_distance;
        int   last_sign;
        int   n_sign_cunt;
        int   sel_max_pri_id;

        //vector<Obstacle> v_obstacle_orbbec;
        //list<int>  list_sign_cun_orbbec;
        //int   n_sign_cunt_orbbec;
        //int   turning_angle_orbbec;

    };

    struct Path_map_info
    {
        ros::Publisher plan_path_on_map_pub;
        ros::Publisher plan_path_on_map_dynamic_pub;
        ros::Publisher move_path_on_map_pub;
        ros::Publisher path_odom_combined_pub;
        nav_msgs::Path           move_path_on_map;
        nav_msgs::Path           plan_path_on_map;
        nav_msgs::Path           plan_path_on_map_dynamic;
        nav_msgs::Path           odom_path;
        geometry_msgs::PoseStamped current_gps_pose;
        geometry_msgs::PoseStamped pose_Stamped;
        double current_direction;
        double global_direction;
        double yaw_with_north;
        double yaw_with_north_globle;
    }; 



    enum State
    {
        INIT,
        START,
        PERCEPTION,
        PLAN,
        MOVE,
        END
    };
    enum Move_State
    {
        MOVE_CURVE,
        MOVE_FORWARD,
        MOVE_TURN,
        MOVE_TURN_LEFT,
        MOVE_TURN_RIGHT,
        MOVE_LEFT_BACKWARD,
        MOVE_RIGHT_BACKWARD,
        MOVE_BACKWARD,
        STOP_AND_WAIT

    };

    enum Plan_State
    {
        PLAN_INIT,
        FREEZE,
        UNFREEZE,
        WAIT
    };

    enum Sensor_mode
    {
        LASER_MODE,
        GPS_LASER
    };

    struct Point
    {
        double x;
        double y;

        Point (double _x, double _y)
        {
            x=_x;
            y=_y;
        };
        Point()
        {};
    };

    struct Twist_def
    {
        double v;
        double w;

        Twist_def (double _v, double _w)
        {
            v=_v;
            w=_w;
        };
        Twist_def()
        {};
    };

    enum Color : int
    {
        //BLACK,
        GREEN,
        YELLOW,
        LIGHT_RED,
        LIGHT_GREEN,
        LIGHT_YELLOW,
        LIGHT_CYAN,
        LIGHT_MAGENTA,
        CYAN,
        MAGENTA,
        RED,
        WHITE,
        LIGHT_BLUE,
        BLUE,
        GRAY
    };


   static  std_msgs::ColorRGBA createColorRGBA( int name_Color)
    {
        std_msgs::ColorRGBA color_rgba;
        color_rgba.r = COLOR_VALUE_MIN;
        color_rgba.g = COLOR_VALUE_MIN;
        color_rgba.b = COLOR_VALUE_MIN;
        color_rgba.a = COLOR_VALUE_MAX;
        switch (name_Color)
        {
        
        case GREEN:
            color_rgba.g = COLOR_VALUE_MAX;
            break;
        case YELLOW:
            color_rgba.r = COLOR_VALUE_MAX;
            color_rgba.g = COLOR_VALUE_MAX;
            break;
        case LIGHT_RED:
            color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.g = COLOR_VALUE_LIGHT_LOW;
            color_rgba.b = COLOR_VALUE_LIGHT_LOW;
            break;
        case LIGHT_GREEN:
            color_rgba.r = COLOR_VALUE_LIGHT_LOW;
            color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.b = COLOR_VALUE_LIGHT_LOW;
            break;
        case LIGHT_YELLOW:
            color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.b = COLOR_VALUE_LIGHT_LOW;
            break;
        case CYAN:
            color_rgba.g = COLOR_VALUE_MAX;
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case MAGENTA:
            color_rgba.r = COLOR_VALUE_MAX;
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case LIGHT_MAGENTA:
            color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.g = COLOR_VALUE_LIGHT_LOW;
            color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
            break;
        case WHITE:
            color_rgba.r = COLOR_VALUE_MAX;
            color_rgba.g = COLOR_VALUE_MAX;
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case LIGHT_CYAN:
            color_rgba.r = COLOR_VALUE_LIGHT_LOW;
            color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
            break;
        case LIGHT_BLUE:
            color_rgba.r = COLOR_VALUE_LIGHT_LOW;
            color_rgba.g = COLOR_VALUE_LIGHT_LOW;
            color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
            break;
        case RED:
            color_rgba.r = COLOR_VALUE_MAX;
            break;
        case BLUE:
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case GRAY:
            color_rgba.r = COLOR_VALUE_MEDIAN;
            color_rgba.g = COLOR_VALUE_MEDIAN;
            color_rgba.b = COLOR_VALUE_MEDIAN;
            break;
        default:
            color_rgba.r = COLOR_VALUE_MAX;
            break;
        /*default:
            color_rgba.a = COLOR_VALUE_MIN; // hide color from view
            break;*/
        }
        return color_rgba;
    }


    const double ultrasound_angular=15;
    const double ultrasound_Distance[36]= {0.6,0.5,0.4,0.3,0.2,0.1,0.6,0.5,0.4,0.3,0.2,0.1,0.6,0.5,0.4,0.3,0.2,0.1,0.6,0.5,0.4,0.3,0.2,0.1,
    0.6,0.5,0.4,0.3,0.2,0.1,0.6,0.5,0.4,0.3,0.2,0.1};
    const double angular_min_FR[6]={0,0,0,
                        atan2((ultrasound_Distance[3]+0.04),(0.13-ultrasound_Distance[3]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        atan2((ultrasound_Distance[4]+0.04),(0.13-ultrasound_Distance[4]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        atan2((ultrasound_Distance[5]+0.04),(0.13-ultrasound_Distance[5]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90};
    const double angular_max_FR[6]={atan2((ultrasound_Distance[0]+0.04),(0.13+2*ultrasound_Distance[0]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        atan2((ultrasound_Distance[1]+0.04),(0.13+ultrasound_Distance[1]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        atan2((ultrasound_Distance[2]+0.04),(0.13+ultrasound_Distance[2]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        atan2((ultrasound_Distance[3]+0.04),(0.13+ultrasound_Distance[3]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        atan2((ultrasound_Distance[4]+0.04),(0.13+ultrasound_Distance[4]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        atan2((ultrasound_Distance[5]+0.04),(0.13+ultrasound_Distance[5]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90};

    const double angular_min_R[6]={atan2((ultrasound_Distance[0]+0.088),(0.21-ultrasound_Distance[0]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[1]+0.088),(0.21-ultrasound_Distance[1]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[2]+0.088),(0.21-ultrasound_Distance[2]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[3]+0.088),(0.21-ultrasound_Distance[3]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[4]+0.088),(0.21-ultrasound_Distance[4]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[5]+0.088),(0.21-ultrasound_Distance[5]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180};
    const double angular_max_R[6]={atan2((ultrasound_Distance[0]+0.088),(0.21+2*ultrasound_Distance[0]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[1]+0.088),(0.21+ultrasound_Distance[1]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[2]+0.088),(0.21+ultrasound_Distance[2]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[3]+0.088),(0.21+ultrasound_Distance[3]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[4]+0.088),(0.21+ultrasound_Distance[4]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[5]+0.088),(0.21+ultrasound_Distance[5]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180};

    const double angular_min_BR[6]={-atan2(ultrasound_Distance[0]+0.29,0.058)*180/M_PI-90,
                        -atan2(ultrasound_Distance[1]+0.29,0.058)*180/M_PI-90,
                        -atan2(ultrasound_Distance[2]+0.29,0.058)*180/M_PI-90,
                        -atan2(ultrasound_Distance[3]+0.29,0.058)*180/M_PI-90,
                        -atan2(ultrasound_Distance[4]+0.29,0.058)*180/M_PI-90,
                        -atan2(ultrasound_Distance[5]+0.29,0.058)*180/M_PI-90};
    /*const double angular_min_BR[6]={-atan2((ultrasound_Distance[0]+0.29),(0.058+ultrasound_Distance[0]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        -atan2((ultrasound_Distance[1]+0.29),(0.058+ultrasound_Distance[1]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        -atan2((ultrasound_Distance[2]+0.29),(0.058+ultrasound_Distance[2]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        -atan2((ultrasound_Distance[3]+0.29),(0.058+ultrasound_Distance[3]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        -atan2((ultrasound_Distance[4]+0.29),(0.058+ultrasound_Distance[4]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90,
                        -atan2((ultrasound_Distance[5]+0.29),(0.058+ultrasound_Distance[5]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-90};
    const double angular_max_BR[6]={atan2((ultrasound_Distance[0]+0.058),(0.29+2*ultrasound_Distance[0]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[1]+0.058),(0.29+ultrasound_Distance[1]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[2]+0.058),(0.29+ultrasound_Distance[2]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[3]+0.058),(0.29+ultrasound_Distance[3]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[4]+0.058),(0.29+ultrasound_Distance[4]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180,
                        atan2((ultrasound_Distance[5]+0.058),(0.29+ultrasound_Distance[5]*tan(ultrasound_angular/180.0*M_PI)))*180/M_PI-180};*/
    const double angular_min_FL[6]={-angular_min_FR[0],-angular_min_FR[1],-angular_min_FR[2],-angular_min_FR[3],-angular_min_FR[4],-angular_min_FR[5]};
    const double angular_max_FL[6]={-angular_max_FR[0],-angular_max_FR[1],-angular_max_FR[2],-angular_max_FR[3],-angular_max_FR[4],-angular_max_FR[5]};
    const double angular_min_L[6]={-angular_min_R[0],-angular_min_R[1],-angular_min_R[2],-angular_min_R[3],-angular_min_R[4],-angular_min_R[5]};
    const double angular_max_L[6]={-angular_max_R[0],-angular_max_R[1],-angular_max_R[2],-angular_max_R[3],-angular_max_R[4],-angular_max_R[5]};
    const double angular_min_BL[6]={-angular_min_BR[0],-angular_min_BR[1],-angular_min_BR[2],-angular_min_BR[3],-angular_min_BR[4],-angular_min_BR[5]};
    //const double angular_max_BL[6]={-angular_max_BR[0],-angular_max_BR[1],-angular_max_BR[2],-angular_max_BR[3],-angular_max_BR[4],-angular_max_BR[5]};
    const double angular_ave_ultrasound[36]={
        (angular_min_FL[0]+angular_max_FL[0])/2,(angular_min_FL[1]+angular_max_FL[1])/2,(angular_min_FL[2]+angular_max_FL[2])/2,
        (angular_min_FL[3]+angular_max_FL[3])/2,(angular_min_FL[4]+angular_max_FL[4])/2,(angular_min_FL[5]+angular_max_FL[5])/2,
        (angular_min_L[0]+angular_max_L[0])/2,(angular_min_L[1]+angular_max_L[1])/2,(angular_min_L[2]+angular_max_L[2])/2,
        (angular_min_L[3]+angular_max_L[3])/2,(angular_min_L[4]+angular_max_L[4])/2,(angular_min_L[5]+angular_max_L[5])/2,
        angular_min_BL[0],angular_min_BL[1],angular_min_BL[2],angular_min_BL[3],angular_min_BL[4],angular_min_BL[5],
        angular_min_BR[0],angular_min_BR[1],angular_min_BR[2],angular_min_BR[3],angular_min_BR[4],angular_min_BR[5],
        (angular_min_R[0]+angular_max_R[0])/2,(angular_min_R[1]+angular_max_R[1])/2,(angular_min_R[2]+angular_max_R[2])/2,
        (angular_min_R[3]+angular_max_R[3])/2,(angular_min_R[4]+angular_max_R[4])/2,(angular_min_R[5]+angular_max_R[5])/2,
        (angular_min_FR[0]+angular_max_FR[0])/2,(angular_min_FR[1]+angular_max_FR[1])/2,(angular_min_FR[2]+angular_max_FR[2])/2,
        (angular_min_FR[3]+angular_max_FR[3])/2,(angular_min_FR[4]+angular_max_FR[4])/2,(angular_min_FR[5]+angular_max_FR[5])/2};
   /*const int    LASRE_BEAM = 360;
    //const int    LASRE_BEAM_ORBBCE = 60;
    //const double  C_LASRE_THRESHOLD = 7.5;  //4 5 6  6.5  7
    //const double max_vel_x=0.3;
    //const double min_vel_x=-0.3;
    const double VEL_X_01=0.1;
    const double max_rot_vel50=5;
    const double min_rot_vel12=1.2;
    const double ROT_VEL_01=0.1;
    //const double acc_lim_x= 4;
    //const double acc_lim_th= 4;
    const double C_linear_speed=0.5;
    const double C_angular_speed=0.5;
    //const double C_ranges_max_set=4;
    //const double C_ORBBCE_ranges_max_set=12;
    //const double C_ranges_max=3.5;
    //const double C_ORBBCE_ranges_max=8;

    //const double  WIDTH = 0.14;
    //const double  LONG = 0.14;
    //const double  C_BODY = 0.14;

    //     设定前向目标距离为 d_goal， 当前方向与目标方向偏差 angle_bias 单位是度
    //     与目标距离越近曲率半径r越小，反之越大，当前方向与目标方向角度偏差越大曲率半径越小，反之越大
    //     即 与目标距离大小成正比，与目标角度大小成反比，即 r ~ d_goal/angle_bias 即 r= C* d_goal/angle_bias （C为常数）
    //       （angle_bias单位是度时令C=45, angle_bias单位是弧度时令C=4/pi;  若想走的弧线更直更接近直线，需要正大C值。
    //     
    //      r= C* d_goal/angle_bias=V/w   当V固定时，w=V*d_goal/d_goal/45;


// 两轮差速   
    //const double  wheel_interval=0.126;          //两轮子间距
    //const double  wheel_diameter=0.065;          //轮子的直径*/
    //     Vr=右轮速度，   Vl=左轮速度，  wheel_interval=两轮子间距  
    //     V= 整体线速度，  w=整体角速度， r=曲率半径
    //
    //     V= (Vr + Vl)/2 
    //
    //     w= (Vr - Vl)/wheel_interval
    //
    //     r= V/w = wheel_interval*(Vr + Vl)/(2(Vr - Vl))
    //
    //     由整体线速度和整体角速度得到左右轮速度 
    //
    //     Vr= V + 0.5*w*wheel_interval
    //      
    //     Vl= V - 0.5*w*wheel_interval
    //

    //    speed_ratio (unit: m/encode): 编码器脉冲值与电机轮胎位移的一个比例系数，（就是电机转过一个编码脉冲这个适合电机轮胎走过多少距离）
    //根据cmd_vel计算出编码器要变化的增量通过串口发送给stm32， delt_encode_right和delt_encode_left：
    //delt_encode_right= Vr * encode_sampling_time / speed_ratio
    //delt_encode_left= Vl * encode_sampling_time / speed_ratio
    //串口接受来自stm32的编码器的脉冲数delt_encode_Right，delt_encode_Left，计算得到里程计左右轮增量delta_d_left,delta_d_right:
    //delta_d_right= delt_encode_Right* speed_ratio;
    //delta_d_left = delt_encode_Left* speed_ratio;
    // 里程计计算出的左右轮速度,    delta_theta是角度变化量
    //wheel_left_speed= delt_encode_left * speed_ratio / encode_sampling_time；
    //wheel_left_speed= delt_encode_left * speed_ratio / encode_sampling_time；
    //delta_d= (delta_d_right + delta_d_left)/2;
    //delta_theta= (delta_d_right - delta_d_left)/wheel_interval;
    //delt_x= delta_d* cos(oriention + *0.5delta_theta);
    //delt_y= delta_d* sin(oriention + *0.5delta_theta);
    // 里程计数据的计算，用来发布
    //position_x+= delt_x;
    //position_y-= delt_y;
    //oriention+ = delta_theta;
    // velocity_linear = delta_d/ encode_sampling_time;
    // velocity_angular= delta_theta/ encode_sampling_time;

// 四轮阿克曼底盘,前两个轮从动轮有一个舵机，后两个轮驱动轮
//https://blog.csdn.net/qq_40870689/article/details/87971282
//https://blog.csdn.net/yangziluomu/article/details/101225620
//https://blog.csdn.net/qq_24706659/article/details/88342626
    //     由整体线速度和整体角速度得到左右轮速度,   theta是前轮转角,d_FB是前后轮间距离，wheel_interval是左右轮之间距离
    //     r= v/w = d_FB/tan(theta); 
    //     w= v * tan(theta)/ d_FB ;
    //     Vr= v + 0.5*w*wheel_interval=  v + 0.5* v * tan(theta)/ d_FB *wheel_interval 
    //     Vl= v - 0.5*w*wheel_interval=  v - 0.5* v * tan(theta)/ d_FB *wheel_interval


// PID ： 
//https://blog.csdn.net/as480133937/article/details/89508034
//https://blog.csdn.net/qq_15063463/article/details/82498869
// e(k)= input_target- feedback_current；T：采样周期，Ti积分时间，Td微分时间，    
//位置型 :        Kp:比例系数;  Ki=Kp* T/Ti 积分系数; Kd=Kp* Td/T 微分系数 
//  u(k)= Kp*[e(k) + 1/Ti*T*(e(0)+...+e(k)) + Td*(e(k)-e(k-1))/T ] = Kp*e(k) +  Ki(e(0)+...+e(k)) + Kd*(e(k)-e(k-1));


// 增量型
// u(k)=  Kp*e(k) +  Ki(e(0)+...+e(k)) + Kd*(e(k)-e(k-1));
// u(k-1)=  Kp*e(k-1) +  Ki(e(0)+...+e(k-1)) + Kd*(e(k-1)-e(k-2));
// delta_u(k) =  u(k)- u(k-1);
// delta_u(k) =  Kp*[e(k) - e(k-1)] + Ki*e(k) + Kd*[e(k) -2*e(k-1) + e(k-2)];


    #endif

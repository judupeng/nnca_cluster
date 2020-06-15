#ifndef __GENRRATE_BEZIER_H__
#define __GENRRATE_BEZIER_H__


#include "nnca_cluster/struct_define.h"
#include <nav_msgs/Path.h>

class Generate_bezier
{
    private:
    
    std::vector<double> m_u;
    std::vector<double> m_curvature;
    Inflate_info m_inflate_info;
    double comapre_range_left;
    double comapre_range_right;
    double comapre_range;
    double n_step_len;
    int   comapre_id;
    int   min_top_left_id;
    int   min_top_right_id;
    int   min_left_down_id;
    int   min_right_down_id;
    int   n_order_bezier;
    int   n_insert_points_num; //不包含两个端点
    bool  init_flag;
    
    public:
    nav_msgs::Path          m_bezier_controlpoint_path;
    nav_msgs::Path          m_bezier_curve_path;
    sensor_msgs::LaserScan  m_laser_msg;
    std::vector<Point> m_bezier_curve;
    std::vector<Point> m_control_points;
    std::vector<Twist_def> m_twist_def;
    public:
    Generate_bezier(Car_info  car_info);
    virtual ~Generate_bezier();
    void gen_control_points_5(Obstacle &select_obstacle,const sensor_msgs::LaserScan& msg);
    void judge_fun(double laser_id,int &left_id,int &right_id);
    void find_min_range_id(int select_);
    void set_control_points(std::vector<Point> control_points);
    void cal_bezier_path();
    void gen_bezier(int insert_points_num=20);
    void cal_twist(double v=0.2);
    void gen_bezier_1();
    void gen_bezier_2();
    void gen_bezier_3();
    void gen_bezier_4();
    void gen_bezier_5();

    Point get_point(double r,int indx_angle);
    double mean_vw(int begin_id,int end_id,std::vector<Twist_def> m_twist_def,int select_);




};

#endif

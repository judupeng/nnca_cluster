#include "nnca_cluster/generate_bezier.h"

/*  使用
//设置规则、优先级
        laser_priority_Set(v_obstacle,v_obstacle_orbbec);
        sel_max_pri_Obstacle=cmd_Set(v_obstacle,v_obstacle_orbbec,twist_cmd);
        if (abs(sel_max_pri_Obstacle.angular)< 75)
        {
            Generate_bezier bezier;
            bezier.gen_control_points_5(sel_max_pri_Obstacle,scan_msg_processed);
            bezier.gen_bezier();
            bezier.cal_bezier_path();
            bezier.cal_twist(0.1);
            bezier_controlpoint_path_pub.publish(bezier.m_bezier_controlpoint_path);
            bezier_curve_path_pub.publish(bezier.m_bezier_curve_path);
            //twist_cmd2.linear.x=bezier.mean_vw(6,bezier.m_twist_def.size()-1,bezier.m_twist_def,0);
            //twist_cmd2.angular.z= bezier.mean_vw(6,bezier.m_twist_def.size()-1,bezier.m_twist_def,1);
            twist_cmd2.linear.x=bezier.m_twist_def[2].v;
            twist_cmd2.angular.z=bezier.m_twist_def[2].w;
            ROS_INFO("linear_speed=%3.2f,angular_speed=%3.2f",twist_cmd2.linear.x,twist_cmd2.angular.z);
        }
*/


Generate_bezier::Generate_bezier(Car_info  car_info)
:init_flag(0),
n_order_bezier(5)
{
    m_inflate_info.inflate_l_r=(car_info.WIDTH*10 < 3.5)? car_info.WIDTH*10:3.5 ;
    m_inflate_info.inflate_front=(car_info.WIDTH*20 < 3.5)? car_info.WIDTH*20:3.5 ;
    m_inflate_info.inflate_back=0;
    m_inflate_info.left_top_lsaer_id= int(atan2(m_inflate_info.inflate_l_r,m_inflate_info.inflate_front)*180/M_PI);
    m_inflate_info.right_top_lsaer_id= 360-m_inflate_info.left_top_lsaer_id;
    m_inflate_info.left_lsaer_id= 90;
    m_inflate_info.right_lsaer_id= 270;
}

Generate_bezier::~Generate_bezier()
{

}


Point Generate_bezier::get_point(double r,int indx_angle)
{
    Point point_tmp;
    point_tmp.x=r* cos(indx_angle*M_PI*1.0f/180);
    point_tmp.y=r* sin(indx_angle*M_PI*1.0f/180);
    return point_tmp;
}

void Generate_bezier::judge_fun(double laser_id,int &left_id,int &right_id)
{
    if(m_laser_msg.ranges[laser_id] <= comapre_range_left)
    {
        left_id=laser_id;
        comapre_range_left=m_laser_msg.ranges[laser_id];
    }
    if(m_laser_msg.ranges[360-laser_id] <= comapre_range_right)
    {
        right_id=360-laser_id;
        comapre_range_right=m_laser_msg.ranges[360-laser_id];
    }  
}

void Generate_bezier::find_min_range_id(int select_)
{

    switch (select_)
    {
        case 0 :
            //分别计算左下、右下的最近距离
            comapre_id=floor(m_inflate_info.left_lsaer_id/2)+1;
            comapre_range=m_inflate_info.inflate_l_r/sin(comapre_id*M_PI*1.0f/180);
            comapre_range_left=comapre_range;
            comapre_range_right=comapre_range;
            for(int i=comapre_id; i<m_inflate_info.left_lsaer_id; i++)
                judge_fun(i,min_left_down_id,min_right_down_id); 
            break ;
        case 1 :
            //分别计算左上、右上的最近距离
            comapre_id=floor(m_inflate_info.left_lsaer_id/4)+1;
            comapre_range=m_inflate_info.inflate_l_r/sin(comapre_id*M_PI*1.0f/180);
            comapre_range_left=comapre_range;
            comapre_range_right=comapre_range;
            for(int i=1; i<floor(m_inflate_info.left_lsaer_id/2) ;i++)
                judge_fun(i,min_top_left_id,min_top_right_id); 
            break ;
    }

}



void Generate_bezier::gen_control_points_5(Obstacle  &select_obstacle,const sensor_msgs::LaserScan& msg)
{
    Point point_tmp3;
    Point point_tmp4;
    Point point_tmp5;
    Point point_tmp6;
    m_laser_msg=msg;

    min_top_left_id=-1;
    min_top_right_id=-1;
    min_left_down_id=-1;
    min_right_down_id=-1;
    // 当前位置控制点
    m_control_points.push_back(Point(0,0));
//分别计算左下、右下的最近距离

    find_min_range_id(0);

    // 第2和第3个控制点
    if(min_left_down_id!=-1 && min_right_down_id!=-1)
    {
        if(m_laser_msg.ranges[min_left_down_id]<m_laser_msg.ranges[min_right_down_id])
            point_tmp3=get_point(m_laser_msg.ranges[min_right_down_id],min_right_down_id);
        else
            point_tmp3=get_point(m_laser_msg.ranges[min_left_down_id],min_left_down_id);
    }
    if(min_left_down_id==-1 && min_right_down_id!=-1)
        point_tmp3=get_point(comapre_range,comapre_id);
    if(min_left_down_id!=-1 && min_right_down_id==-1)
        point_tmp3=get_point(comapre_range,360-comapre_id);
    if(min_left_down_id==-1 && min_right_down_id==-1)
    {
        if(select_obstacle.angular>0)
            point_tmp3=get_point(comapre_range,360-comapre_id);
        else
            point_tmp3=get_point(comapre_range,comapre_id);
    }

    m_control_points.push_back(Point(point_tmp3.x,0));
    point_tmp3.y= point_tmp3.y/2;
    m_control_points.push_back(point_tmp3);
//分别计算左上、右上的最近距离
    find_min_range_id(1);
    
// 第4和第5个控制点
    if(min_top_left_id!=-1 && min_top_right_id!=-1)
    {
        point_tmp4=get_point(m_laser_msg.ranges[min_top_left_id],min_top_left_id);
        point_tmp5=get_point(m_laser_msg.ranges[min_top_right_id],min_top_right_id);
        point_tmp4.y= point_tmp4.y/2;
        point_tmp5.y= point_tmp5.y/2;
        if(point_tmp4.x <point_tmp5.x)
        {
            m_control_points.push_back(point_tmp4);
            m_control_points.push_back(point_tmp5);
        }
        else
        {
            m_control_points.push_back(point_tmp5);
            m_control_points.push_back(point_tmp4);
        }
    }
    
    if(min_top_left_id==-1 && min_top_right_id!=-1)
    {
            point_tmp4=get_point(m_laser_msg.ranges[min_top_right_id],min_top_right_id);
            point_tmp5=get_point(comapre_range,comapre_id);
            point_tmp4.y= point_tmp4.y/2;
            point_tmp5.y= point_tmp5.y/2;
            m_control_points.push_back(point_tmp4);
            m_control_points.push_back(point_tmp5);
    }
    
    
    if(min_top_left_id!=-1 && min_top_right_id==-1)
    {
            point_tmp4=get_point(m_laser_msg.ranges[min_top_left_id],min_top_left_id);
            point_tmp5=get_point(comapre_range,comapre_id);
            point_tmp4.y= point_tmp4.y/2;
            point_tmp5.y= point_tmp5.y/2;
            m_control_points.push_back(point_tmp4);            
            m_control_points.push_back(point_tmp5);
    }

    if(min_top_left_id==-1 && min_top_right_id==-1)
    {
        if(select_obstacle.angular>0)
        {
            point_tmp4=get_point(comapre_range,360-comapre_id);
            point_tmp5=get_point(comapre_range,comapre_id);
            point_tmp4.y= point_tmp4.y/2;
            point_tmp5.y= point_tmp5.y/2;
            m_control_points.push_back(point_tmp4);
            m_control_points.push_back(point_tmp5);
        }
        else
        {
            point_tmp4=get_point(comapre_range,comapre_id);
            point_tmp5=get_point(comapre_range,360-comapre_id);
            point_tmp4.y= point_tmp4.y/2;
            point_tmp5.y= point_tmp5.y/2;
            m_control_points.push_back(point_tmp4);
            m_control_points.push_back(point_tmp5);
        }
    }

// 第6个控制点
// 目标位置控制点
    comapre_id=select_obstacle.laser_ids[floor(select_obstacle.n_lasers/2)];
    comapre_range=m_laser_msg.ranges[comapre_id];
    point_tmp6=get_point(comapre_range,comapre_id);
    m_control_points.push_back(point_tmp6);
    init_flag=1;

}

void Generate_bezier::cal_twist(double v)
{
    Twist_def wist_;
    double omega_;
    double v_;
    for (int i =0; i< m_curvature.size();i++)
    {   
        v_=v;
        omega_=v*m_curvature[i];
        while (abs(omega_)>=1.2)
        {
            v-=0.005;
            omega_=v*m_curvature[i];
        }

        m_twist_def.push_back(Twist_def(v,omega_));
    }
}



void Generate_bezier::cal_bezier_path()
{
    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.header.frame_id=m_laser_msg.header.frame_id;
    pose_tmp.header.stamp=m_laser_msg.header.stamp;
    for (int i=0; i<m_control_points.size(); i++)
    {
        pose_tmp.pose.position.x=m_control_points[i].x;
        pose_tmp.pose.position.y=m_control_points[i].y;
        m_bezier_controlpoint_path.poses.push_back(pose_tmp);
    }
    for (int i=0; i<m_bezier_curve.size(); i++)
    {
        pose_tmp.pose.position.x=m_bezier_curve[i].x;
        pose_tmp.pose.position.y=m_bezier_curve[i].y;
        m_bezier_curve_path.poses.push_back(pose_tmp);
    }
    m_bezier_controlpoint_path.header.frame_id="base_scan";
    m_bezier_curve_path.header.frame_id="base_scan";

}


void Generate_bezier::set_control_points(std::vector<Point> control_points)
{
    m_control_points=control_points;
    n_order_bezier=control_points.size()-1;
    init_flag=1;
}


void Generate_bezier::gen_bezier(int insert_points_num)
{

    Point point_tmp;

    if (n_order_bezier+1 != m_control_points.size() )
    {
        std::cout << "error !"<<"  给出控制点数不一致 ！" << std::endl;
        return ;
    }
    if (init_flag!=1)
    {
        std::cout << "没有设置控制点！" << std::endl;
        return ;
    }

    n_insert_points_num=insert_points_num;
    n_step_len= 1.0/(n_insert_points_num+1);
    for(int i=0; i< n_insert_points_num+2 ;i++)
        m_u.push_back(i*n_step_len);

    switch (n_order_bezier)
    {
        case 5:
            gen_bezier_5();
            break ;

        case 4:
            gen_bezier_4();
            break ;

        case 3:
            gen_bezier_3();
            break ;

        case 2:
            gen_bezier_2();
            break ;

        case 1:
            gen_bezier_1();
            break ;

        default:
            std::cout <<"更高阶数曲线功能未实现"<< std::endl;
            break ;
    }

}

double  Generate_bezier::mean_vw(int begin_id,int end_id,std::vector<Twist_def> m_twist_def,int select_)
{
    double sum=0;
    if (select_==0)
        for(int i=begin_id; i<=end_id; i++ )
            sum+=m_twist_def[i].v;
    else
        for(int i=begin_id; i<=end_id; i++ )
            sum+=m_twist_def[i].w;
    
    return sum/(end_id-begin_id+1);
}

void Generate_bezier::gen_bezier_5()
{
    Point point_tmp;
    double del_xy[n_insert_points_num+2];
    double del_xy2[n_insert_points_num+2];
    double curvature;
    for(int i=0; i<n_insert_points_num+2; i++ )
    {
        point_tmp.x= pow(1-m_u[i],5)*m_control_points[0].x + 5*pow(1-m_u[i],4)*m_u[i]* m_control_points[1].x +
                10*pow(1-m_u[i],3)*pow(m_u[i],2)* m_control_points[2].x + 10*pow(1-m_u[i],2)*pow(m_u[i],3)* m_control_points[3].x
                + 5*(1-m_u[i])*pow(m_u[i],4)* m_control_points[4].x + pow(m_u[i],5)* m_control_points[5].x;
            
        point_tmp.y= pow(1-m_u[i],5)*m_control_points[0].y + 5*pow(1-m_u[i],4)*m_u[i]* m_control_points[1].y +
                10*pow(1-m_u[i],3)*pow(m_u[i],2)* m_control_points[2].y + 10*pow(1-m_u[i],2)*pow(m_u[i],3)* m_control_points[3].y
                + 5*(1-m_u[i])*pow(m_u[i],4)* m_control_points[4].y + pow(m_u[i],5)* m_control_points[5].y;

        m_bezier_curve.push_back(point_tmp);
    }

     //计算一阶导数
     del_xy[n_insert_points_num+1]=0;
     for(int i=0; i<n_insert_points_num+1; i++)
     {
        del_xy[i]=(m_bezier_curve[i+1].y-m_bezier_curve[i].y)/(m_bezier_curve[i+1].x-m_bezier_curve[i].x);
     }

    //计算二阶导数
     del_xy2[0]=0;
     del_xy2[n_insert_points_num+1]=0;
     for(int i=1; i<n_insert_points_num+1; i++)
     {
        del_xy2[i]=(m_bezier_curve[i+1].y-2*m_bezier_curve[i].y+m_bezier_curve[i-2].y )/
            pow(0.5*(m_bezier_curve[i+1].x-m_bezier_curve[i-1].x),2);
     }

     //计算曲率
     for(int i=0; i<=n_insert_points_num+1; i++)
     {
         curvature=del_xy2[i]/pow(1+ pow(del_xy[i], 2), 1.5) ;
         m_curvature.push_back(-curvature);
     }
    std::cout <<"5阶 bezier ！"<< std::endl;
}

void Generate_bezier::gen_bezier_4()
{
    Point point_tmp;
    for(int i=0; i<n_insert_points_num+2; i++ )
    {
        point_tmp.x= pow(1-m_u[i],4)* m_control_points[0].x + 4*pow(1-m_u[i],3)*m_u[i]* m_control_points[1].x + 
            6*pow(1-m_u[i],2)*pow(m_u[i],2)* m_control_points[2].x+ 4*(1-m_u[i])*pow(m_u[i],3)* m_control_points[3].x + 
                pow(m_u[i],4)* m_control_points[4].x;
            
        point_tmp.y= pow(1-m_u[i],4)* m_control_points[0].y + 4*pow(1-m_u[i],3)*m_u[i]* m_control_points[1].y + 
            6*pow(1-m_u[i],2)*pow(m_u[i],2)* m_control_points[2].y+ 4*(1-m_u[i])*pow(m_u[i],3)* m_control_points[3].y + 
                pow(m_u[i],4)* m_control_points[4].y;

        m_bezier_curve.push_back(point_tmp);
    }

    std::cout <<"4阶 bezier ！"<< std::endl;
}

void Generate_bezier::gen_bezier_3()
{
    Point point_tmp;
    for(int i=0; i<n_insert_points_num+2; i++ )
    {
        point_tmp.x=pow(1-m_u[i],3)*m_control_points[0].x + 3*pow(1-m_u[i],2)*m_u[i]* m_control_points[1].x +
                3*(1-m_u[i])*pow(m_u[i],2)* m_control_points[2].x + pow(m_u[i],3)* m_control_points[3].x;
            
        point_tmp.y=pow(1-m_u[i],3)*m_control_points[0].y + 3*pow(1-m_u[i],2)*m_u[i]* m_control_points[1].y +
                3*(1-m_u[i])*pow(m_u[i],2)* m_control_points[2].y + pow(m_u[i],3)* m_control_points[3].y;

        m_bezier_curve.push_back(point_tmp);
    }

    std::cout <<"3阶 bezier ！"<< std::endl;
}

void Generate_bezier::gen_bezier_2()
{
    Point point_tmp;
    for(int i=0; i<n_insert_points_num+2; i++ )
    {
        point_tmp.x= pow(1-m_u[i],2)* m_control_points[0].x +
                2*(1-m_u[i])*m_u[i]* m_control_points[1].x + pow(m_u[i],2)* m_control_points[2].x;
            
        point_tmp.y= pow(1-m_u[i],2)* m_control_points[0].y +
                2*(1-m_u[i])*m_u[i]* m_control_points[1].y + pow(m_u[i],2)* m_control_points[2].y;

        m_bezier_curve.push_back(point_tmp);
    }
    std::cout <<"2阶 bezier ！"<< std::endl;

}

void Generate_bezier::gen_bezier_1()
{
    Point point_tmp;
    for(int i=0; i<n_insert_points_num+2; i++ )
    {
        point_tmp.x= (1-m_u[i])* m_control_points[0].x +m_u[i]* m_control_points[1].x;
            
        point_tmp.y= (1-m_u[i])* m_control_points[0].y +m_u[i]* m_control_points[1].y;

        m_bezier_curve.push_back(point_tmp);
    }

    std::cout <<"1阶 bezier ！"<< std::endl;

}
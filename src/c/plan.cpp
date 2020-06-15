#include "nnca_cluster/plan.h"

Plan::Plan(Plan_info plan_info,nav_msgs::Path &plan_path_on_map,geometry_msgs::PoseStamped current_gps_pose, 
                    Car_info  car_info,Laser_info  laser_info_tmp)
    :PLAN_STATE(PLAN_INIT),
    MOVE_STATE(STOP_AND_WAIT),
    SENSOR_MODE(LASER_MODE),
    n_sign_cunt(0),
    n_sign_sum(0),
    turning_angle(30),
    turning_angle_orbbec(15),
    COUNT(5)
    {
        twist_def=Twist_def(0,0);
        m_plan_info.lock_angle=plan_info.lock_angle;
        m_plan_info.yaw_with_north=plan_info.yaw_with_north;
        m_plan_info.is_current_direction=plan_info.is_current_direction;
        m_plan_info.is_global_direction=plan_info.is_global_direction;
        m_plan_path_on_map=plan_path_on_map;
        m_current_gps_pose=current_gps_pose;
        laser_info=laser_info_tmp;
        inflate_info.inflate_l_r=car_info.WIDTH*1.1;
        inflate_info.inflate_front=car_info.WIDTH*2.5;
        inflate_info.inflate_back=0;
        inflate_info.left_top_lsaer_id= int(atan2(inflate_info.inflate_l_r,inflate_info.inflate_front)*180/M_PI);
        inflate_info.right_top_lsaer_id= 360-inflate_info.left_top_lsaer_id;
        inflate_info.left_lsaer_id= 90;
        inflate_info.right_lsaer_id= 270;
    }

void Plan::find_max_pri(vector<Obstacle>& v_obstacle_tmp)
{
    double sel_angular=180;
    double max_pri=0;
    //查找最大优先级，同等优先级选择夹角较小的
    for (int i=0; i< v_obstacle_tmp.size(); i++)
    {
        if(v_obstacle_tmp[i].pri> max_pri || (v_obstacle_tmp[i].pri== max_pri && abs(v_obstacle_tmp[i].angular) < abs(sel_angular)) )
        {
            max_pri=v_obstacle_tmp[i].pri;
            sel_angular=v_obstacle_tmp[i].angular;
            n_sel_max_pri_id=i;
        }
    }
}

void Plan::find_max_pri_gpsmode(vector<Obstacle>& v_obstacle_tmp,int lock_mode)
{
    double max_pri=-0.0;
    double min_angular=180.0;
    int sel_max_n_pri_id[5]={0};
    double sel_angular,deltx,delty;
    nav_msgs::Path plan_path_on_map_temp;

    plan_path_on_map_temp=m_plan_path_on_map;
    pro_closest_route(plan_path_on_map_temp,&m_current_gps_pose);
    //Point3 closest_point = find_closest_ponit3(m_current_gps_pose,plan_path_on_map_temp);
    vector<geometry_msgs::PoseStamped>().swap(m_plan_path_on_map_dynamic.poses);
    m_plan_path_on_map_dynamic.poses.push_back(m_current_gps_pose);
    m_plan_path_on_map_dynamic.poses.push_back(plan_path_on_map_temp.poses.front());
   	//m_plan_path_on_map_dynamic.poses.push_back(closest_point);
    m_plan_path_on_map_dynamic.header.frame_id="map";

    if(plan_path_on_map_temp.poses.size()< 1)
    {
        ROS_INFO("arrieve at goal !");
        return ;
    }

    deltx=plan_path_on_map_temp.poses[0].pose.position.x-m_current_gps_pose.pose.position.x;
    delty=plan_path_on_map_temp.poses[0].pose.position.y-m_current_gps_pose.pose.position.y;
    if(pow(deltx,2)+pow(delty,2)<= 4)
    {
        ROS_INFO("arrieve at small target  !");
        vector<geometry_msgs::PoseStamped>::iterator k = m_plan_path_on_map.poses.begin();
        m_plan_path_on_map.poses.erase(k);
        return ;
    }   
    m_global_direction=atan2(delty,deltx)*180/M_PI;
    if (m_global_direction>=0)
        m_global_direction=m_global_direction-180;
    else
        m_global_direction=m_global_direction+180;
    sel_angular=m_global_direction-m_plan_info.yaw_with_north;
    if (sel_angular > 180)
        sel_angular=360-sel_angular;
    else if(sel_angular < -180)
        sel_angular=360+sel_angular;                

    
    min_angular=180.0;
    switch (lock_mode)
    {
        case 0:
             priorityQueuePri(sel_max_n_pri_id,5, v_obstacle_tmp);//找出找出前5个最大优先级
        case 1:
             priorityQueuePriLockAngle(sel_max_n_pri_id,5,m_plan_info.lock_angle, v_obstacle_tmp);//找出找出-75～75角度内的前5个最大优先级
    }
    max_pri=v_obstacle_tmp[sel_max_n_pri_id[0]].pri;
    for (int i=0;i<5;i++ )
    {
        double tmp = abs(v_obstacle_tmp[sel_max_n_pri_id[i]].angular-sel_angular);
        if (tmp < min_angular)
        {
            min_angular=tmp;
            n_sel_max_pri_id=sel_max_n_pri_id[i];
        }
    }
    if(v_obstacle_tmp[n_sel_max_pri_id].dmax==laser_info.C_ranges_max_set &&  v_obstacle_tmp[n_sel_max_pri_id].n_lasers>= 30)
        v_obstacle_tmp[n_sel_max_pri_id].angular=sel_angular;

}


void Plan::find_lock_max_pri(vector<Obstacle>& v_obstacle_tmp)
{
    double sel_angular=180;
    double max_pri=0;
    //查找最大优先级，同等优先级选择夹角较小的
    for (int i=0; i< v_obstacle_tmp.size(); i++)
    {
        if(abs(v_obstacle_tmp[i].angular) < m_plan_info.lock_angle)
            if(v_obstacle_tmp[i].pri> max_pri || (v_obstacle_tmp[i].pri== max_pri && abs(v_obstacle_tmp[i].angular) < abs(sel_angular)) )
            {
                max_pri=v_obstacle_tmp[i].pri;
                sel_angular=v_obstacle_tmp[i].angular;
                n_sel_max_pri_id=i;
            }
    }
}

void Plan::update_sign(vector<Obstacle>& v_obstacle_tmp)
{
    if (v_obstacle_tmp[n_sel_max_pri_id].angular_speed >=0)
        m_list_sign_cun.push_back(1);
    else
        m_list_sign_cun.push_back(-1);
    n_sign_sum=accumulate(m_list_sign_cun.begin(),m_list_sign_cun.end(),0);
}


bool  Plan::inflate_detect(const sensor_msgs::LaserScan& msg,sensor_msgs::LaserScan &inflat_msg_left,
        sensor_msgs::LaserScan  &inflat_msg_right, Inflate_info &inflate_info)
{
    double detect_range_front=0;
    double detect_range_left_right=0;
    vector<float>().swap(inflat_msg_left.ranges);
    vector<float>().swap(inflat_msg_left.intensities);
    vector<float>().swap(inflat_msg_right.ranges);
    vector<float>().swap(inflat_msg_right.intensities);
    
    detect_range_front=inflate_info.inflate_front;
    if(msg.ranges[0] <= detect_range_front)
    {
        inflat_msg_left.ranges.push_back(msg.ranges[0]);
        inflat_msg_left.intensities.push_back(0);
    }

    for(int i=1; i<inflate_info.left_top_lsaer_id ;i++)
    {
        detect_range_front=inflate_info.inflate_front/cos(i*laser_info.angle_increment);
        if(msg.ranges[i] <= detect_range_front)
        {
           inflat_msg_left.ranges.push_back(msg.ranges[i]);
           inflat_msg_left.intensities.push_back(i);
        }
        if(msg.ranges[360-i] <= detect_range_front)
        {
           inflat_msg_right.ranges.push_back(msg.ranges[360-i]);
           inflat_msg_right.intensities.push_back(i);
        }     
    }
     
    for(int i=inflate_info.left_top_lsaer_id; i<inflate_info.left_lsaer_id ;i++)
    {
        detect_range_left_right=inflate_info.inflate_l_r/cos((90-i)*laser_info.angle_increment);
        if(msg.ranges[i] <= detect_range_left_right)
        {
           inflat_msg_left.ranges.push_back(msg.ranges[i]);
           inflat_msg_left.intensities.push_back(i);
        }
        if(msg.ranges[360-i] <= detect_range_left_right)
        {
           inflat_msg_right.ranges.push_back(msg.ranges[360-i]);
           inflat_msg_right.intensities.push_back(i);
        }     
    }   
    if(inflat_msg_left.intensities.size()>0 || inflat_msg_right.intensities.size()>0)
        return 1;
    return 0;
};




void Plan::set_unfreeze_move_stete(vector<Obstacle>& v_obstacle_tmp,sensor_msgs::LaserScan &scan_msg)
{

    double angular_abs;
    sensor_msgs::LaserScan inflat_msg_left;
    sensor_msgs::LaserScan inflat_msg_right;
    //更新符号
    m_list_sign_cun.pop_front();
    update_sign(v_obstacle_tmp);
    // 
    if ((n_sign_sum*v_obstacle_tmp[n_sel_max_pri_id].angular_speed>=0) || abs(v_obstacle_tmp[n_sel_max_pri_id].angular)<= 5)
    {
        if (v_obstacle_tmp[n_sel_max_pri_id].pri > 1)
        {
            m_last_sel_Obstacle=v_obstacle_tmp[n_sel_max_pri_id];
            //近距离有障碍物
            if(inflate_detect(scan_msg,inflat_msg_left,inflat_msg_right,inflate_info))
            { 
                if(inflat_msg_left.intensities.size()>0 && inflat_msg_right.intensities.size()==0)
                {
                    MOVE_STATE=MOVE_TURN_RIGHT;
                    PLAN_STATE=UNFREEZE;
                    ROS_INFO("Obstacles on left!!" );
                    return ;
                }    
                if(inflat_msg_left.intensities.size()==0 && inflat_msg_right.intensities.size()>0)
                {
                    MOVE_STATE=MOVE_TURN_LEFT;
                    PLAN_STATE=UNFREEZE;
                    ROS_INFO("Obstacles on right!" );
                    return ;
                }
                if(inflat_msg_left.intensities.size()>0 && inflat_msg_right.intensities.size()>0)
                {
                    MOVE_STATE=STOP_AND_WAIT;
                    PLAN_STATE=UNFREEZE;
                    ROS_INFO("Obstacles on both sides!!" );
                    return ;
                }
            }
            angular_abs=abs(v_obstacle_tmp[n_sel_max_pri_id].angular);
            if (angular_abs<= turning_angle )//走曲线
            {
                MOVE_STATE=MOVE_CURVE;
                PLAN_STATE=FREEZE;
                ROS_INFO("going ");
            }
            else  //仅仅转弯
            {
                MOVE_STATE=MOVE_TURN;
                PLAN_STATE=UNFREEZE;
                ROS_INFO("turning" );
            }
            return ;
        }
        
        MOVE_STATE=STOP_AND_WAIT;
        PLAN_STATE=UNFREEZE;
    }
}

void Plan::set_freeze_move_stete(vector<Obstacle>& v_obstacle_tmp,sensor_msgs::LaserScan &scan_msg)
{

    double angular_abs;
    sensor_msgs::LaserScan inflat_msg_left;
    sensor_msgs::LaserScan inflat_msg_right;
    //更新符号
    m_list_sign_cun.pop_front();
    update_sign(v_obstacle_tmp);
    // 
    if ((n_sign_sum*v_obstacle_tmp[n_sel_max_pri_id].angular_speed>=0) || abs(v_obstacle_tmp[n_sel_max_pri_id].angular)<= 5)
    {
        if (v_obstacle_tmp[n_sel_max_pri_id].pri > 1)
        {
            m_last_sel_Obstacle=v_obstacle_tmp[n_sel_max_pri_id];
            //近距离有障碍物
            if(inflate_detect(scan_msg,inflat_msg_left,inflat_msg_right,inflate_info))
            { 
                if(inflat_msg_left.intensities.size()>0 && inflat_msg_right.intensities.size()==0)
                {
                    MOVE_STATE=MOVE_TURN_RIGHT;
                    PLAN_STATE=FREEZE;
                    ROS_INFO("Obstacles on left!!" );
                    return ;
                }    
                if(inflat_msg_left.intensities.size()==0 && inflat_msg_right.intensities.size()>0)
                {
                    MOVE_STATE=MOVE_TURN_LEFT;
                    PLAN_STATE=FREEZE;
                    ROS_INFO("Obstacles on right!" );
                    return ;
                }
                if(inflat_msg_left.intensities.size()>0 && inflat_msg_right.intensities.size()>0)
                {
                    MOVE_STATE=STOP_AND_WAIT;
                    PLAN_STATE=UNFREEZE;
                    ROS_INFO("Obstacles on both sides!!" );
                    return ;
                }
            }
            angular_abs=abs(v_obstacle_tmp[n_sel_max_pri_id].angular);
            if (angular_abs<= turning_angle )//走曲线
            {
                MOVE_STATE=MOVE_CURVE;
                PLAN_STATE=FREEZE;
                ROS_INFO("going ");
            }
            else  //仅仅转弯
            {
                MOVE_STATE=MOVE_TURN;
                PLAN_STATE=FREEZE;
                ROS_INFO("turning" );
            }
            return ;
        }
        MOVE_STATE=STOP_AND_WAIT;
        PLAN_STATE=UNFREEZE;
    }
}

void Plan::plan_init(vector<Obstacle>& v_obstacle_tmp)
{

    find_max_pri(v_obstacle_tmp);
    m_last_sel_Obstacle=v_obstacle_tmp[n_sel_max_pri_id];
    n_sign_cunt++; //等待3帧数据到后再使用
    if (n_sign_cunt>10000)
        n_sign_cunt=n_sign_cunt-9990;
    if (n_sign_cunt<=COUNT)
    {
        update_sign(v_obstacle_tmp);
        PLAN_STATE=PLAN_INIT;
        return ;
    }
    PLAN_STATE=UNFREEZE;
}

void Plan::unfreeze_plan(vector<Obstacle>& v_obstacle_tmp,sensor_msgs::LaserScan &scan_msg,ros::Publisher &plan_path_pub)
{    

    if (m_plan_info.is_current_direction==1 && m_plan_info.is_global_direction==1)
    {
        ROS_INFO("GPS mode !");
        find_max_pri_gpsmode( v_obstacle_tmp,0);
        plan_path_pub.publish(m_plan_path_on_map_dynamic);
    }
    else
    {
        //查找最大优先级，同等优先级选择夹角较小的
        ROS_INFO("normal mode !");
        find_max_pri(v_obstacle_tmp);
    }
    set_unfreeze_move_stete(v_obstacle_tmp,scan_msg);
    ROS_INFO("unfreeze_plan !");
}



void Plan::freeze_plan(vector<Obstacle>& v_obstacle_tmp,sensor_msgs::LaserScan &scan_msg,ros::Publisher &plan_path_pub)
{

    if (m_plan_info.is_current_direction==1 && m_plan_info.is_global_direction==1)
    {
        ROS_INFO("GPS mode !");
        find_max_pri_gpsmode( v_obstacle_tmp,1);
        plan_path_pub.publish(m_plan_path_on_map_dynamic);
    }
    else
    {
        ROS_INFO("normal mode !");
        find_lock_max_pri(v_obstacle_tmp);
    }

    set_freeze_move_stete(v_obstacle_tmp,scan_msg);
    ROS_INFO("freeze_plan !");
}
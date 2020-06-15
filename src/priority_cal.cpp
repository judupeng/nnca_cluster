#include "nnca_cluster/priority_cal.h"
#include<queue>
//  不锁定角度时找前5个优先级
void priorityQueuePri(int (&sel_max_n_pri_id)[5],int n,vector<Obstacle>& v_obstacle_tmp)
{
    priority_queue <Obstacle> q;
    for (int i=0; i<v_obstacle_tmp.size();i++)
        q.push(v_obstacle_tmp[i]);
    int i=0;
    while(!q.empty())
    {
        q.pop();
        sel_max_n_pri_id[i]=q.top().class_id-1;
        i++;
        if(i>=n)
            break;
    }
}
//  锁定角度时找前5个优先级
void priorityQueuePriLockAngle(int (&sel_max_n_pri_id)[5],int n,double lock_angle,vector<Obstacle>& v_obstacle_tmp)
{
    priority_queue <Obstacle> q;
    for (int i=0; i<v_obstacle_tmp.size();i++)
        if(abs(v_obstacle_tmp[i].angular) < lock_angle)
            q.push(v_obstacle_tmp[i]);
    int i=0;
    while(!q.empty())
    {
        q.pop();
        sel_max_n_pri_id[i]=q.top().class_id-1;
        i++;
        if(i>=n)
            break;
    }
}

void gpsMode(int lock_state,Obstacle_info & obstacle_info,SensorsInfo sensors_info,Path_map_info &path_map_info)  //lock_state=1 未锁定， 2 锁定
{
    ROS_INFO("GPS mode !");
    vector<Obstacle>& v_obstacle_tmp= obstacle_info.v_obstacle;
    double sel_angular=180;
    double min_angular=180;
    int sel_max_n_pri_id[5]={0};
    obstacle_info.max_pri=0;
    nav_msgs::Path plan_path_on_map_temp;

    plan_path_on_map_temp = path_map_info.plan_path_on_map;
    pro_closest_route(plan_path_on_map_temp,&path_map_info.current_gps_pose);
    vector<geometry_msgs::PoseStamped>().swap(path_map_info.plan_path_on_map_dynamic.poses);
    path_map_info.plan_path_on_map_dynamic.poses.push_back(path_map_info.current_gps_pose);
    path_map_info.plan_path_on_map_dynamic.poses.push_back(plan_path_on_map_temp.poses.front());
    path_map_info.plan_path_on_map_dynamic.header.frame_id="map";
    path_map_info.plan_path_on_map_dynamic_pub.publish(path_map_info.plan_path_on_map_dynamic);
    if(plan_path_on_map_temp.poses.size()>=1)
    {
        double deltx=plan_path_on_map_temp.poses[0].pose.position.x-path_map_info.current_gps_pose.pose.position.x;
        double delty=plan_path_on_map_temp.poses[0].pose.position.y-path_map_info.current_gps_pose.pose.position.y;
        if(pow(deltx,2)+pow(delty,2)> 4)
        {
            path_map_info.global_direction=atan2(delty,deltx)*D_180_div_PI;
            int sgn = (path_map_info.global_direction>= 0) ? (-1) : (1);
            path_map_info.global_direction=path_map_info.global_direction + sgn*180;
            sel_angular=path_map_info.global_direction-path_map_info.yaw_with_north;

            if (sel_angular > 180)
                sel_angular=360-sel_angular;
            else if(sel_angular < -180)
                sel_angular=360+sel_angular;
        }
        else
        {
            ROS_INFO("arrieve at small target  !");
            vector<geometry_msgs::PoseStamped>::iterator k = path_map_info.plan_path_on_map.poses.begin();
            path_map_info.plan_path_on_map.poses.erase(k);
        }                    
    }
    else
    {
        ROS_INFO("arrieve at goal !");
    }

    if (lock_state ==2 )
        priorityQueuePriLockAngle(sel_max_n_pri_id,5,obstacle_info.lock_angle, v_obstacle_tmp);//找出找出-75～75角度内的前5个最大优先级
    else
        priorityQueuePri(sel_max_n_pri_id,5, v_obstacle_tmp);//找出找出前5个最大优先级

    for (int i=0;i<5;i++ )
    {
        double tmp = abs(v_obstacle_tmp[sel_max_n_pri_id[i]].angular-sel_angular);
        if (tmp < min_angular)
        {
            min_angular=tmp;
            obstacle_info.sel_max_pri_id=sel_max_n_pri_id[i];
            obstacle_info.max_pri=v_obstacle_tmp[obstacle_info.sel_max_pri_id].pri;
        }
    }
    if(v_obstacle_tmp[obstacle_info.sel_max_pri_id].dmax==sensors_info.laser_info.C_ranges_max_set &&  v_obstacle_tmp[obstacle_info.sel_max_pri_id].n_lasers>= 30)
        v_obstacle_tmp[obstacle_info.sel_max_pri_id].angular=sel_angular;
}

void  normalMode(int lock_state, Obstacle_info & obstacle_info)  //lock_state=1 未锁定， 2 锁定
{
    ROS_INFO("normal mode !");
    vector<Obstacle>& v_obstacle_tmp= obstacle_info.v_obstacle;
    int sel_max_n_pri_id[5]={0};
    double sel_angular=180;
    double angular_tmp1,angular_tmp2;
    /*
    if (lock_state ==2 )
        priorityQueuePriLockAngle(sel_max_n_pri_id,5,obstacle_info.lock_angle, v_obstacle_tmp);//找出找出-75～75角度内的前5个最大优先级
    else
        priorityQueuePri(sel_max_n_pri_id,5, v_obstacle_tmp);//找出找出前5个最大优先级 */
 

    if (lock_state ==2 )
        for (int i=0; i< v_obstacle_tmp.size(); i++) //找出最大优先级
        {
            if(abs(v_obstacle_tmp[i].angular) < obstacle_info.lock_angle)  //找出lock_angle -75～75角度内的最大优先级
                if(v_obstacle_tmp[i].pri> obstacle_info.max_pri || (v_obstacle_tmp[i].pri== obstacle_info.max_pri && (abs(v_obstacle_tmp[i].angular) < abs(sel_angular))))
                {
                    obstacle_info.max_pri=v_obstacle_tmp[i].pri;
                    obstacle_info.sel_max_pri_id=i;
                }
        }
    else
        for (int i=0; i< v_obstacle_tmp.size(); i++) //找出最大优先级
        {
            if(v_obstacle_tmp[i].pri> obstacle_info.max_pri || (v_obstacle_tmp[i].pri== obstacle_info.max_pri && (abs(v_obstacle_tmp[i].angular) < abs(sel_angular))))
            {
                obstacle_info.max_pri=v_obstacle_tmp[i].pri;
                obstacle_info.sel_max_pri_id=i;
                sel_angular=v_obstacle_tmp[i].angular;
            }
        }
}

//深度相机的模拟激光数据用
void  normalModeOrbbec( Obstacle_info & obstacle_info)
{
    normalMode(1, obstacle_info) ;
}
 //等带几帧数据
int  signCount(Obstacle_info & obstacle_info)
{
    vector<Obstacle>& v_obstacle_tmp= obstacle_info.v_obstacle;
    obstacle_info.n_sign_cunt++; //等待7帧数据到后再使用
    if (obstacle_info.n_sign_cunt>10000)
        obstacle_info.n_sign_cunt=obstacle_info.n_sign_cunt-9990;
    if (obstacle_info.n_sign_cunt<= 7)
    {
        obstacle_info.last_sel_Obstacle=v_obstacle_tmp[obstacle_info.sel_max_pri_id];
        int sgn = (v_obstacle_tmp[obstacle_info.sel_max_pri_id].angular >=0) ? (1) : (-1);
        obstacle_info.list_sign_cun.push_back(sgn);
        return 0;
    }
    else
        return 1;
}


double distance_Weigh(Obstacle &obstacle_temp)
{
   return 0.25*obstacle_temp.dmin+0.25*sqrt(obstacle_temp.din2)+ 0.25*obstacle_temp.dmax
    +0.25*obstacle_temp.pri_A;
}


void pri_Set(Obstacle &obstacle_temp,double dmin,Car_info car_info)   //优先级计算入口 （有反射数据的）
 {

    static   double ind_0=(car_info.WIDTH < 0.15) ? 2*car_info.WIDTH : 0.15+car_info.WIDTH;

    if (ind_0 >= dmin ||  ind_0*ind_0/4 >= obstacle_temp.din2  )
    //if (ind_0 >= dmin)
    {
        obstacle_temp.pri =0;
        return;
    }

    if (ind_0 < dmin &&   dmin <= ind_0+1.5)  //1.5米内的优先级15个
    {
        for (int j=0; j< 15 ;j++)
           if(ind_0+0.1*j < dmin &&   dmin <= ind_0+0.1*(j+1))
           {
                obstacle_temp.pri=j+1;
                return;
           }
    }

    if(ind_0+1.5 < dmin &&   dmin <= ind_0+3.5)//1.5～3.5米内的优先级10个
    {
        for (int j=0; j< 10 ;j++)
           if(ind_0+1.5+0.2*j < dmin &&   dmin <= ind_0+1.5+0.2*(j+1))
            {
                obstacle_temp.pri=j+16;
                return;
            }
    }

    if(ind_0+3.5 < dmin &&   dmin <= ind_0+8.5)//3.5～8.5米内的优先级5个
    {
        for (int j=0; j< 5 ;j++)
        if(ind_0+3.5+j < dmin &&   dmin <= ind_0+3.5+(j+1))
        {
            obstacle_temp.pri=j+26;
            return;
        }                
    }

    if(dmin > ind_0+8.5)
        obstacle_temp.pri=31;

}


void pri_Set_MAX_RANGE(Obstacle &obstacle_temp,std::vector<double > threshold_arr)   //优先级计算入口  (无反射数据的优先级)
{
    double pri_A = obstacle_temp.pri_A;
    if (pri_A <= threshold_arr[1])
    {
         obstacle_temp.pri=0;
         return ;
    }

    if (pri_A > threshold_arr[22])
    {
        obstacle_temp.pri=26+22;
        return ;
    }

    for (int i=1; i<22 ;i++)
        if(threshold_arr[i] < pri_A && pri_A <= threshold_arr[i+1])
        {
            obstacle_temp.pri=26+i;
            return ;
        }
}


bool  inflate_detect(const sensor_msgs::LaserScan& msg,sensor_msgs::LaserScan &inflat_msg_left,
        sensor_msgs::LaserScan  &inflat_msg_right, Inflate_info &inflate_info,Laser_info laser_info)
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
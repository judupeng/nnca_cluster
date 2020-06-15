#include "nnca_cluster/nnc_fun.h"

void filter_mid_max3(sensor_msgs::LaserScan& msg,Laser_info laser_info,int _case)
{
    sensor_msgs::LaserScan msg_tmp=msg;
    int n_size=msg.ranges.size();
    if (_case==1) //选择中间值
    {
        for (int i=1; i<n_size-1; i++)
        {
            if (msg_tmp.ranges[i]==laser_info.C_ranges_max_set)
                if(msg_tmp.ranges[i-1]<= msg_tmp.ranges[i+1])
                    msg.ranges[i]=msg_tmp.ranges[i+1];
                else 
                    msg.ranges[i]=msg_tmp.ranges[i-1];

        }
        // 头
        if (msg_tmp.ranges[0]==laser_info.C_ranges_max_set)
            if(msg_tmp.ranges[n_size-1]<= msg_tmp.ranges[1])
                msg.ranges[0]=msg_tmp.ranges[1];
            else 
                msg.ranges[0]=msg_tmp.ranges[n_size-1];
        // 尾
        if (msg_tmp.ranges[n_size-1]==laser_info.C_ranges_max_set)
            if(msg_tmp.ranges[n_size-2]<= msg_tmp.ranges[0])
                msg.ranges[n_size-1]=msg_tmp.ranges[0];
            else 
                msg.ranges[n_size-1]=msg_tmp.ranges[n_size-2];
    }
    else //选择最小值
    {
        for (int i=1; i<n_size-1; i++)
        {
            if (msg_tmp.ranges[i]==laser_info.C_ranges_max_set)
                if(msg_tmp.ranges[i-1]<= msg_tmp.ranges[i+1])
                    msg.ranges[i]=msg_tmp.ranges[i-1];
                else 
                    msg.ranges[i]=msg_tmp.ranges[i+1];
        }
        // 头
        if (msg_tmp.ranges[0]==laser_info.C_ranges_max_set)
            if(msg_tmp.ranges[n_size-1]<= msg_tmp.ranges[1])
                msg.ranges[0]=msg_tmp.ranges[n_size-1];
            else 
                msg.ranges[0]=msg_tmp.ranges[1];
        // 尾
        if (msg_tmp.ranges[n_size-1]==laser_info.C_ranges_max_set)
            if(msg_tmp.ranges[n_size-2]<= msg_tmp.ranges[0])
                msg.ranges[n_size-1]=msg_tmp.ranges[n_size-2];
            else 
                msg.ranges[n_size-1]=msg_tmp.ranges[0];
    }
}

void filter_mid3(sensor_msgs::LaserScan& msg)
{
    sensor_msgs::LaserScan msg_tmp=msg;
    int n_size=msg.ranges.size();
    int id1=0;
    int id2=0;
     for (int i=1; i<n_size-1; i++)
     {
         id1=(msg_tmp.ranges[i-1]<= msg_tmp.ranges[i])? (i-1): i;
         id2=(msg_tmp.ranges[i]<= msg_tmp.ranges[i+1])? i: i+1;
         msg.ranges[i]=(msg_tmp.ranges[id1]<= msg_tmp.ranges[id2])? msg_tmp.ranges[id2]: msg_tmp.ranges[id1];
     }
     // 头
     id1=(msg_tmp.ranges[n_size-1]<= msg_tmp.ranges[0])? (n_size-1): 0;
     id2=(msg_tmp.ranges[0]<= msg_tmp.ranges[1])? 0:1;
     msg.ranges[0]=(msg_tmp.ranges[id1]<= msg_tmp.ranges[id2])? msg_tmp.ranges[id2]: msg_tmp.ranges[id1];
     // 尾
     id1=(msg_tmp.ranges[n_size-1]<= msg_tmp.ranges[n_size-2])? (n_size-1):(n_size-2);
     id2=(msg_tmp.ranges[0]<= msg_tmp.ranges[n_size-1])? 0:n_size-1;
     msg.ranges[n_size-1]=(msg_tmp.ranges[id1]<= msg_tmp.ranges[id2])? msg_tmp.ranges[id2]: msg_tmp.ranges[id1];
}


void pre_process( sensor_msgs::LaserScan& msg,Laser_info& laser_info)
{
    int n_size=msg.ranges.size();
    for (int i=0; i<n_size; i++)  //  处理 inf  和 0 情况下极端数据，设置为C_ranges_max_set
    {
        if (msg.ranges[i]> laser_info.C_ranges_max  || isinf(msg.ranges[i])==true || msg.ranges[i]<=0)
            msg.ranges[i]=laser_info.C_ranges_max_set;
        if(msg.ranges[i]< laser_info.range_min &&  msg.ranges[i]>0)
            msg.ranges[i]=laser_info.range_min;
    }
    filter_mid_max3(msg,laser_info,1);  //  对 C_ranges_max_set的数据做3阶中值滤波
    filter_mid3(msg);                    //  对 所有数据做3阶中值滤波
    laser_info.msg=msg;
}

double square_sum(double x,double x2,double y,double y2)
{
    return pow(x-x2,2) + pow(y-y2,2);
}

double rms_cal(double x,double x2,double y,double y2)
{
    return sqrt(pow(x-x2,2) + pow(y-y2,2));
}

double cal_obstacle_din(int temp_start_id, int temp_end_id,Laser_info laser_info)
{
    sensor_msgs::LaserScan laser_tmp=laser_info.msg;
    double x,y,x2,y2;
    double angle;
    angle=cal_Angle(temp_start_id,laser_tmp);
    y = laser_tmp.ranges[temp_start_id]* sin(angle);
    x = laser_tmp.ranges[temp_start_id]* cos(angle);
    angle=cal_Angle(temp_end_id,laser_tmp);
    y2 = laser_tmp.ranges[temp_end_id]* sin(angle);
    x2 = laser_tmp.ranges[temp_end_id]* cos(angle);

    return square_sum( x, x2, y, y2);
}


double cal_pria(Obstacle &obstacle_temp,int temp_start_id,int temp_end_id,Laser_info laser_info )
{   
    sensor_msgs::LaserScan laser_tmp=laser_info.msg;
    double pri_A_temp=0;
   
    if (laser_tmp.ranges[temp_start_id] <= laser_tmp.ranges[temp_end_id])
        pri_A_temp = laser_tmp.ranges[temp_start_id]* sin(laser_info.angle_increment)*(obstacle_temp.laser_ids.size()+1);
    else
        pri_A_temp = laser_tmp.ranges[temp_end_id]* sin(laser_info.angle_increment)*(obstacle_temp.laser_ids.size()+1);
    return pri_A_temp;
}


double cal_Angle(int laser_id, const sensor_msgs::LaserScan& laser_tmp)
{
    double angle_tmp=laser_tmp.angle_min+laser_tmp.angle_increment*laser_id;
    return (angle_tmp <=  M_PI) ? angle_tmp:angle_tmp-D_2_div_PI;  //D_2_div_PI
}
double cal_Mid_Angle(Obstacle &obstacle, const sensor_msgs::LaserScan& laser_tmp)
{
    int mid=obstacle.laser_ids[floor((obstacle.n_lasers-1)/2.0)];
    return cal_Angle(mid,laser_tmp);
}

void calculating_statistics_head_tail(Laser_info laser_info,Obstacle &obstacle_head,Obstacle &obstacle_tail ,int select_)
{
    sensor_msgs::LaserScan laser_tmp=laser_info.msg;
    double temp_angular=0;
    double min_temp,max_temp;
    double pri_A_temp=0;
    int  temp_start_id=0;
    int  temp_end_id=0;
    int size_head= obstacle_head.n_lasers;
    int size_tail= obstacle_tail.n_lasers;
    temp_start_id= obstacle_head.start_id;
    temp_end_id=   obstacle_head.laser_ids[obstacle_head.n_lasers-1];
    switch(select_)
    {
        case 0 :

            //第一个类是空气，开始ID不是0，最后一个类是障碍物   laser_tmp.angle_min+ laser_tmp.angle_increment*id
            obstacle_head.dmax=laser_info.C_ranges_max_set;
            obstacle_head.dmin=laser_info.C_ranges_max_set;
            obstacle_head.din2=cal_obstacle_din(temp_start_id-1,temp_end_id+1, laser_info);
            obstacle_head.dave= (laser_tmp.ranges[temp_start_id-1]+laser_tmp.ranges[temp_end_id+1])/2;
            obstacle_head.angular=cal_Mid_Angle(obstacle_head,laser_tmp)*D_180_div_PI;
              //  只计算无返回数据的参数pri_A，不计算有返回数据的参数pri_A
            obstacle_head.pri_A=cal_pria(obstacle_head,temp_start_id-1,temp_end_id+1, laser_info);

            //最后一个类
            temp_start_id= obstacle_tail.start_id;
            temp_end_id=  obstacle_tail.laser_ids[obstacle_tail.n_lasers-1];
            obstacle_tail.dmax=*max_element( laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0], laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0]+obstacle_tail.n_lasers);
            obstacle_tail.dmin=*min_element( laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0], laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0]+obstacle_tail.n_lasers);
            obstacle_tail.din2=cal_obstacle_din(temp_start_id,temp_end_id, laser_info);
            obstacle_tail.dave= (obstacle_tail.dmax+obstacle_tail.dmin)/2;
            if(laser_tmp.ranges[temp_start_id-1]==laser_info.C_ranges_max_set || laser_tmp.ranges[temp_end_id+1]==laser_info.C_ranges_max_set)
                obstacle_tail.pri_A=cal_pria(obstacle_tail,temp_start_id,temp_end_id, laser_info);
            else
                obstacle_tail.pri_A=cal_pria(obstacle_tail,temp_start_id-1,temp_end_id+1, laser_info);

            obstacle_tail.angular=cal_Mid_Angle(obstacle_tail,laser_tmp)*D_180_div_PI;

            break;

        case 1 ://第一个类是障碍物，开始ID不是0，最后一个类是障碍物或空气
            max_temp=*max_element(laser_tmp.ranges.begin()+obstacle_head.laser_ids[0], laser_tmp.ranges.end());
            min_temp=*min_element(laser_tmp.ranges.begin()+obstacle_head.laser_ids[0], laser_tmp.ranges.end());
            obstacle_head.dmax=*max_element(laser_tmp.ranges.begin(), laser_tmp.ranges.begin()+obstacle_head.laser_ids[obstacle_head.n_lasers-1]);  
            obstacle_head.dmin=*min_element(laser_tmp.ranges.begin(), laser_tmp.ranges.begin()+obstacle_head.laser_ids[obstacle_head.n_lasers-1]);
            if(obstacle_head.dmax < max_temp)
                obstacle_head.dmax=max_temp;
            if(obstacle_head.dmin > min_temp)
                obstacle_head.dmin=min_temp;
            obstacle_head.din2=cal_obstacle_din(temp_start_id,temp_end_id, laser_info);
            obstacle_head.dave= (obstacle_head.dmax+obstacle_head.dmin)/2;
            obstacle_head.angular=cal_Mid_Angle(obstacle_head,laser_tmp)*D_180_div_PI; 

            if(laser_tmp.ranges[temp_start_id-1]==laser_info.C_ranges_max_set || laser_tmp.ranges[temp_end_id+1]==laser_info.C_ranges_max_set)
                obstacle_head.pri_A=cal_pria(obstacle_head,temp_start_id,temp_end_id, laser_info);
            else
                obstacle_head.pri_A=cal_pria(obstacle_head,temp_start_id-1,temp_end_id+1, laser_info);

            //最后一个类是障碍物或空气
            temp_start_id= obstacle_tail.start_id;
            temp_end_id=  obstacle_tail.laser_ids[obstacle_tail.n_lasers-1];
            if (laser_tmp.ranges[obstacle_tail.laser_ids[0]]==laser_info.C_ranges_max_set)// 是空气
            {
                obstacle_tail.dmax=laser_info.C_ranges_max_set;
                obstacle_tail.dmin=laser_info.C_ranges_max_set;
                obstacle_tail.din2=cal_obstacle_din(temp_start_id-1,temp_end_id+1, laser_info);
                obstacle_tail.dave= (laser_tmp.ranges[temp_start_id-1]+laser_tmp.ranges[temp_end_id+1])/2;
                obstacle_tail.pri_A=cal_pria(obstacle_tail,temp_start_id-1,temp_end_id+1, laser_info);
            }
            else// 是障碍物
            {
                obstacle_tail.dmax=*max_element( laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0], laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0]+obstacle_tail.n_lasers);
                obstacle_tail.dmin=*min_element( laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0], laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0]+obstacle_tail.n_lasers);
                obstacle_tail.din2=cal_obstacle_din(temp_start_id,temp_end_id, laser_info);
                obstacle_tail.dave= (obstacle_tail.dmax+obstacle_tail.dmin)/2;
                if(laser_tmp.ranges[temp_start_id-1]==laser_info.C_ranges_max_set || laser_tmp.ranges[temp_end_id+1]==laser_info.C_ranges_max_set)
                    obstacle_tail.pri_A=cal_pria(obstacle_tail,temp_start_id,temp_end_id, laser_info);
                else
                    obstacle_tail.pri_A=cal_pria(obstacle_tail,temp_start_id-1,temp_end_id+1, laser_info);
            }
            obstacle_tail.angular=cal_Mid_Angle(obstacle_tail,laser_tmp)*D_180_div_PI; 

            break;

        case 2 :
            //第一个类空气，且开始ID为0，最后一类是障碍物
            obstacle_head.dmax=laser_info.C_ranges_max_set;
            obstacle_head.dmin=laser_info.C_ranges_max_set;
            obstacle_head.din2=cal_obstacle_din(size_head-1,temp_end_id+1, laser_info);
            obstacle_head.dave= (laser_tmp.ranges[size_head-1]+laser_tmp.ranges[temp_end_id+1])/2;
            obstacle_head.angular=cal_Mid_Angle(obstacle_head,laser_tmp)*D_180_div_PI; 
            obstacle_head.pri_A=cal_pria(obstacle_tail,size_head-1,temp_end_id+1, laser_info);

            //最后一个类是障碍物
            temp_start_id= obstacle_tail.start_id;
            temp_end_id=  obstacle_tail.laser_ids[obstacle_tail.n_lasers-1];
            obstacle_tail.dmax=*max_element( laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0], laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0]+obstacle_tail.n_lasers);  
            obstacle_tail.dmin=*min_element( laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0], laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0]+obstacle_tail.n_lasers);
            obstacle_tail.din2=cal_obstacle_din(temp_start_id,temp_end_id, laser_info);
            obstacle_tail.angular=cal_Mid_Angle(obstacle_tail,laser_tmp)*D_180_div_PI;
            if(laser_tmp.ranges[temp_start_id-1]==laser_info.C_ranges_max_set || laser_tmp.ranges[temp_end_id+1]==laser_info.C_ranges_max_set)
                    obstacle_tail.pri_A=cal_pria(obstacle_tail,temp_start_id,temp_end_id, laser_info);
                else
                    obstacle_tail.pri_A=cal_pria(obstacle_tail,size_tail-1,temp_end_id+1, laser_info);
            break;

        case 3 :
            //第一个类是障碍物，且开始ID为0，最后一类是空气或障碍物
            obstacle_head.dmax=*max_element( laser_tmp.ranges.begin()+obstacle_head.laser_ids[0], laser_tmp.ranges.begin()+obstacle_head.laser_ids[0]+obstacle_head.n_lasers);  
            obstacle_head.dmin=*min_element( laser_tmp.ranges.begin()+obstacle_head.laser_ids[0], laser_tmp.ranges.begin()+obstacle_head.laser_ids[0]+obstacle_head.n_lasers);
            obstacle_head.din2=cal_obstacle_din(temp_start_id,temp_end_id, laser_info);            
            obstacle_head.dave= (obstacle_head.dmax+obstacle_head.dmin)/2;
            obstacle_head.angular=cal_Mid_Angle(obstacle_head,laser_tmp)*D_180_div_PI;
            if(laser_tmp.ranges[temp_start_id-1]==laser_info.C_ranges_max_set || laser_tmp.ranges[temp_end_id+1]==laser_info.C_ranges_max_set)
                obstacle_head.pri_A=cal_pria(obstacle_head,temp_start_id,temp_end_id, laser_info);
            else
                obstacle_head.pri_A=cal_pria(obstacle_head,temp_start_id-1,0, laser_info);
            //最后一个类是空气
            temp_start_id= obstacle_tail.start_id;
            temp_end_id=   obstacle_tail.laser_ids[obstacle_tail.n_lasers-1];
            if (laser_tmp.ranges[obstacle_tail.laser_ids[0]]==laser_info.C_ranges_max_set)// 是空气
            {
                obstacle_tail.dmax=laser_info.C_ranges_max_set;
                obstacle_tail.dmin=laser_info.C_ranges_max_set;
                obstacle_tail.din2=cal_obstacle_din(temp_start_id-1,0, laser_info);
                obstacle_tail.dave=(laser_tmp.ranges[temp_start_id-1]+laser_tmp.ranges[0])/2;
                obstacle_tail.pri_A=cal_pria(obstacle_tail,temp_start_id-1,0, laser_info);
            }
            else// 是障碍物
            {
                obstacle_tail.dmax=*max_element( laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0], laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0]+obstacle_tail.n_lasers);
                obstacle_tail.dmin=*min_element( laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0], laser_tmp.ranges.begin()+obstacle_tail.laser_ids[0]+obstacle_tail.n_lasers);
                obstacle_tail.din2=cal_obstacle_din(temp_start_id,temp_end_id, laser_info);
                obstacle_tail.dave= (obstacle_tail.dmax+obstacle_tail.dmin)/2;
                if(laser_tmp.ranges[temp_start_id-1]==laser_info.C_ranges_max_set || laser_tmp.ranges[temp_end_id+1]==laser_info.C_ranges_max_set)
                    obstacle_tail.pri_A=cal_pria(obstacle_tail,temp_start_id,temp_end_id, laser_info);
                else
                    obstacle_tail.pri_A=cal_pria(obstacle_tail,temp_start_id-1,0, laser_info);
            }
            obstacle_tail.angular=cal_Mid_Angle(obstacle_tail,laser_tmp)*D_180_div_PI;
            break;
    }

}
/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// /// */
void cal_obstacle_din_again(vector<Obstacle>& v_obstacle_tmp,const sensor_msgs::LaserScan& laser_tmp,Laser_info laser_info )
{
    vector<Obstacle>  v_obstacle_tmp2;
    double tmp_din2=0;
    int  temp_start_id=0;
    int  temp_end_id=0;

    v_obstacle_tmp2.push_back(v_obstacle_tmp[v_obstacle_tmp.size()-2]);
    v_obstacle_tmp2.push_back(v_obstacle_tmp[v_obstacle_tmp.size()-1]);
    for(int i=0; i<v_obstacle_tmp.size()-1; i++ )
        v_obstacle_tmp2.push_back(v_obstacle_tmp[i]);
    v_obstacle_tmp2.push_back(v_obstacle_tmp[0]);
    v_obstacle_tmp2.push_back(v_obstacle_tmp[1]);
    
    for(int i=2; i<v_obstacle_tmp2.size()-2; i++ )
    {
            if(v_obstacle_tmp2[i-1].dmin==laser_info.C_ranges_max_set && v_obstacle_tmp2[i+1].dmin==laser_info.C_ranges_max_set)
            {
                temp_start_id=v_obstacle_tmp2[i-2].laser_ids[v_obstacle_tmp2[i-2].n_lasers-1];
                temp_end_id=v_obstacle_tmp2[i+2].laser_ids[0];
                tmp_din2= cal_obstacle_din(temp_start_id,temp_end_id,laser_info);
                v_obstacle_tmp[i-2].din2= (v_obstacle_tmp[i-2].din2 < tmp_din2)? v_obstacle_tmp[i-2].din2: tmp_din2;
                continue;
            }
            if(v_obstacle_tmp2[i-1].dmin==laser_info.C_ranges_max_set && v_obstacle_tmp2[i+1].dmin!=laser_info.C_ranges_max_set)
            {
                temp_start_id=v_obstacle_tmp2[i-2].laser_ids[v_obstacle_tmp2[i-2].n_lasers-1];
                temp_end_id=v_obstacle_tmp2[i+1].laser_ids[0];
                tmp_din2= cal_obstacle_din(temp_start_id,temp_end_id,laser_info);
                v_obstacle_tmp[i-2].din2= (v_obstacle_tmp[i-2].din2 < tmp_din2)? v_obstacle_tmp[i-2].din2: tmp_din2;
                continue;
            }
            if(v_obstacle_tmp2[i-1].dmin!=laser_info.C_ranges_max_set && v_obstacle_tmp2[i+1].dmin==laser_info.C_ranges_max_set)
            {
                temp_start_id=v_obstacle_tmp2[i-1].laser_ids[v_obstacle_tmp2[i-1].n_lasers-1];
                temp_end_id=v_obstacle_tmp2[i+2].laser_ids[0];
                tmp_din2= cal_obstacle_din(temp_start_id,temp_end_id,laser_info);
                v_obstacle_tmp[i-2].din2= (v_obstacle_tmp[i-2].din2 < tmp_din2)? v_obstacle_tmp[i-2].din2: tmp_din2;
                continue;
            }
            if(v_obstacle_tmp2[i-1].dmin!=laser_info.C_ranges_max_set && v_obstacle_tmp2[i+1].dmin!=laser_info.C_ranges_max_set)
            {
                temp_start_id=v_obstacle_tmp2[i-1].laser_ids[v_obstacle_tmp2[i-1].n_lasers-1];
                temp_end_id=v_obstacle_tmp2[i+1].laser_ids[0];
                tmp_din2= cal_obstacle_din(temp_start_id,temp_end_id,laser_info);
                v_obstacle_tmp[i-2].din2= (v_obstacle_tmp[i-2].din2 < tmp_din2)? v_obstacle_tmp[i-2].din2: tmp_din2;
                continue;
            }

    }
    vector<Obstacle>().swap(v_obstacle_tmp2);

}

void calculating_statistics_360(Laser_info laser_info,Obstacle &obstacle_temp)
{
    sensor_msgs::LaserScan laser_tmp=laser_info.msg;
    double temp_angular=0;
    int  temp_start_id=0;
    int  temp_end_id=0;
    temp_start_id= obstacle_temp.start_id;
    temp_end_id=   obstacle_temp.laser_ids[obstacle_temp.n_lasers-1];
    if (laser_tmp.ranges[obstacle_temp.laser_ids[0]]==laser_info.C_ranges_max_set)
    {  
        obstacle_temp.dmax=laser_info.C_ranges_max_set;
        obstacle_temp.dmin=laser_info.C_ranges_max_set;
        obstacle_temp.din2=cal_obstacle_din(temp_start_id-1,temp_end_id+1, laser_info);
        obstacle_temp.dave= (laser_tmp.ranges[temp_start_id-1]+laser_tmp.ranges[temp_end_id+1])/2;
        obstacle_temp.pri_A= cal_pria(obstacle_temp,temp_start_id-1,temp_end_id+1, laser_info );
    }
    else
    {
        obstacle_temp.dmax=*max_element( laser_tmp.ranges.begin()+obstacle_temp.laser_ids[0], laser_tmp.ranges.begin()+obstacle_temp.laser_ids[0]+obstacle_temp.n_lasers);  
        obstacle_temp.dmin=*min_element( laser_tmp.ranges.begin()+obstacle_temp.laser_ids[0], laser_tmp.ranges.begin()+obstacle_temp.laser_ids[0]+obstacle_temp.n_lasers);
        obstacle_temp.din2=cal_obstacle_din(temp_start_id,temp_end_id, laser_info);
        obstacle_temp.dave= (obstacle_temp.dmax+obstacle_temp.dmin)/2;
        if(laser_tmp.ranges[temp_start_id-1]==laser_info.C_ranges_max_set || laser_tmp.ranges[temp_end_id+1]==laser_info.C_ranges_max_set)
             obstacle_temp.pri_A=cal_pria(obstacle_temp,temp_start_id,temp_end_id, laser_info);
        else
            obstacle_temp.pri_A=cal_pria(obstacle_temp,temp_start_id-1,temp_end_id+1, laser_info);
            
    }  

    obstacle_temp.angular=cal_Mid_Angle(obstacle_temp,laser_tmp)*D_180_div_PI;        
}


bool check_new_class(SensorsInfo sensors_info, int indx_laser,int indx_angle,int select_)
{
    Car_info car_info=sensors_info.car_info;
    Laser_info laser_info=sensors_info.laser_info;
    sensor_msgs::LaserScan laser_tmp=laser_info.msg;
    double threshold_2=0;
    double y=0;
    double y2=0;
    double x=0;
    double x2=0;
    double y3=0;
    double x3=0;
    double dr=0;
    double distance=0;
    int n_size=laser_tmp.ranges.size();
    
    switch (select_)
    {
        case 0:
            dr = laser_tmp.ranges[indx_laser]* sin(laser_info.angle_increment);   
            threshold_2 =pow(laser_info.C*dr,2);
            y = laser_tmp.ranges[indx_laser]* sin(cal_Angle(indx_angle,laser_tmp));
            x = laser_tmp.ranges[indx_laser]* cos(cal_Angle(indx_angle,laser_tmp));
            y2 = laser_tmp.ranges[indx_laser+1]* sin(cal_Angle(indx_angle+1,laser_tmp));
            x2 = laser_tmp.ranges[indx_laser+1]* cos(cal_Angle(indx_angle+1,laser_tmp));
            distance=square_sum(x, x2, y, y2);
            return ( distance> threshold_2  && distance>car_info.WIDTH/2) ;

        case 1:
            dr = laser_tmp.ranges[n_size-1]* sin(laser_info.angle_increment);
            threshold_2 =pow(laser_info.C*dr,2);
            y = laser_tmp.ranges[n_size-1]* sin(cal_Angle(n_size-1,laser_tmp));
            x = laser_tmp.ranges[n_size-1]* cos(cal_Angle(n_size-1,laser_tmp));
            y2 = laser_tmp.ranges[0]* sin(cal_Angle(0,laser_tmp));
            x2 = laser_tmp.ranges[0]* cos(cal_Angle(0,laser_tmp));
            distance=square_sum(x, x2, y, y2);
            return ( distance> threshold_2 && distance>car_info.WIDTH/2) ;
            
        case 2:
            y = laser_tmp.ranges[n_size-1]* sin(cal_Angle(n_size-1,laser_tmp));
            x = laser_tmp.ranges[n_size-1]* cos(cal_Angle(n_size-1,laser_tmp));
            y2 = laser_tmp.ranges[0]* sin(cal_Angle(0,laser_tmp));
            x2 = laser_tmp.ranges[0]* cos(cal_Angle(0,laser_tmp));
            y3 = laser_tmp.ranges[n_size-2]* sin(cal_Angle(n_size-2,laser_tmp));
            x3 = laser_tmp.ranges[n_size-2]* cos(cal_Angle(n_size-2,laser_tmp));

            distance=square_sum(x, x2, y, y2);
            return ( distance> square_sum(x, x3, y, y3)  && distance>car_info.WIDTH/2)  ;
    }
}


void nnc_360_laser(vector<Obstacle>& v_obstacle_tmp,SensorsInfo sensors_info)
{
    Obstacle obstacle_temp;
    Car_info car_info=sensors_info.car_info;
    Laser_info laser_info=sensors_info.laser_info;
    sensor_msgs::LaserScan laser_tmp=laser_info.msg;
    int class_id=1;
    int new_flag=1;
    int n_size=laser_tmp.ranges.size();
    
    for (int i=0; i<n_size-1;i++)  // 中间部分聚类
    {
        if (laser_tmp.ranges[i]<=laser_info.C_ranges_max_set)
        {
            if (new_flag==1)
            {
                obstacle_temp.class_id = class_id;
                obstacle_temp.start_id = i;
                vector<int>().swap(obstacle_temp.laser_ids);
                obstacle_temp.laser_ids.push_back(i);

                if ( check_new_class(sensors_info, i,i,0) )
                {
                    class_id++;
                    obstacle_temp.n_lasers=obstacle_temp.laser_ids.size();
                    v_obstacle_tmp.push_back(obstacle_temp);
                }
                else
                {
                    new_flag=0;
                }
            }
            else
            {
                obstacle_temp.laser_ids.push_back(i);
                if (check_new_class(sensors_info, i,i,0))
                {
                    new_flag=1;
                    class_id++;
                    obstacle_temp.n_lasers=obstacle_temp.laser_ids.size();
                    v_obstacle_tmp.push_back(obstacle_temp);
                }
            }
        }
    }
    // 两头聚类
    //最后一个类，以及它是否与第一个类合并
    if(new_flag==0)
    {
        obstacle_temp.laser_ids.push_back(n_size-1);
        obstacle_temp.n_lasers=obstacle_temp.laser_ids.size();
        if (check_new_class(sensors_info, 0,0,1) )
        {
            obstacle_temp.n_lasers=obstacle_temp.laser_ids.size();
            v_obstacle_tmp.push_back(obstacle_temp);
        }
            
        else
        {   
            obstacle_temp.laser_ids.insert(obstacle_temp.laser_ids.end(), v_obstacle_tmp[0].laser_ids.begin(), v_obstacle_tmp[0].laser_ids.end());
            vector<int>().swap(v_obstacle_tmp[0].laser_ids);
            v_obstacle_tmp[0].laser_ids.insert(v_obstacle_tmp[0].laser_ids.end(), obstacle_temp.laser_ids.begin(), obstacle_temp.laser_ids.end());
            v_obstacle_tmp[0].start_id=obstacle_temp.start_id;
            v_obstacle_tmp[0].n_lasers=v_obstacle_tmp[0].laser_ids.size();
        }
    }
    else
    {
        if(check_new_class(sensors_info, 0,0,2))
        {
            obstacle_temp.laser_ids.push_back(n_size-1);
            obstacle_temp.n_lasers=obstacle_temp.laser_ids.size();
            v_obstacle_tmp.push_back(obstacle_temp);
        }
        else
        {
            v_obstacle_tmp[0].laser_ids.insert(v_obstacle_tmp[0].laser_ids.begin(), n_size-1);
            v_obstacle_tmp[0].n_lasers=v_obstacle_tmp[0].laser_ids.size();
            v_obstacle_tmp[0].start_id=n_size-1;
        }
    }
}

void nnc_fun(SensorsInfo sensors_info,vector<Obstacle>& v_obstacle_tmp)  // 聚类入口
{
    Car_info car_info=sensors_info.car_info;
    Laser_info laser_info=sensors_info.laser_info;
    sensor_msgs::LaserScan laser_tmp=laser_info.msg;
    double y=0;
    double y2=0;
    double x=0;
    double x2=0;
    
    if (v_obstacle_tmp.size()!=0 || v_obstacle_tmp.capacity()!=0)
        vector<Obstacle>().swap(v_obstacle_tmp);

    nnc_360_laser(v_obstacle_tmp , sensors_info);// 完成聚类

    // 计算统计量
    double temp_angular;
    int  temp_start_id, temp_end_id;
    int  last_obstacle_id=v_obstacle_tmp.size()-1;
    temp_start_id= v_obstacle_tmp[0].start_id;

    for(int i=1; i<v_obstacle_tmp.size()-1; i++ )
        calculating_statistics_360(laser_info, v_obstacle_tmp[i]); // 统计中间部分
    // 统计两头
    if (laser_tmp.ranges[v_obstacle_tmp[0].laser_ids[0]]==laser_info.C_ranges_max_set && temp_start_id!=0)
    /* 第一个类和最后一个类，第一种情况*/
        calculating_statistics_head_tail(laser_info,v_obstacle_tmp[0] ,v_obstacle_tmp[last_obstacle_id] ,0);
    /* 第一个类和最后一个类，第二种情况*/
    else if (laser_tmp.ranges[v_obstacle_tmp[0].laser_ids[0]]!=laser_info.C_ranges_max_set && temp_start_id!=0)
     //第一个类是障碍物，开始ID不是0，最后一个类是障碍物或空气
       calculating_statistics_head_tail(laser_info,v_obstacle_tmp[0] ,v_obstacle_tmp[last_obstacle_id] ,1);
    /* 第一个类和最后一个类，第三种情况*/
    else if (laser_tmp.ranges[v_obstacle_tmp[0].laser_ids[0]]==laser_info.C_ranges_max_set && temp_start_id==0)
    //第一个类空气，且开始ID为0，最后一类是障碍物
        calculating_statistics_head_tail(laser_info,v_obstacle_tmp[0] ,v_obstacle_tmp[last_obstacle_id] ,2);
    else
      //第一个类是障碍物，且开始ID为0，最后一类是空气
        calculating_statistics_head_tail(laser_info,v_obstacle_tmp[0] ,v_obstacle_tmp[last_obstacle_id] ,3);
    cal_obstacle_din_again(v_obstacle_tmp,laser_tmp, laser_info );
};


//深度相机的模拟激光数据用
void nnc_xxx_laser(vector<Obstacle>& v_obstacle_tmp,const sensor_msgs::LaserScan& laser_tmp,SensorsInfo sensors_info)
{
    int class_id=1;
    int new_flag=1;
    Obstacle obstacle_temp;
    int n_size=60;
    
   // 最近邻聚类 
    for (int i=-30; i<60-30;i++)
    {
        int j=i+180;
        if (new_flag==1)
        {
            obstacle_temp.class_id = class_id;
            obstacle_temp.start_id = i;
            vector<int>().swap(obstacle_temp.laser_ids);
            obstacle_temp.laser_ids.push_back(i);
            if ( check_new_class(sensors_info, j,i,0) )
            {
                class_id++;
                obstacle_temp.n_lasers=obstacle_temp.laser_ids.size();
                v_obstacle_tmp.push_back(obstacle_temp);
            }
            else
            {
                new_flag=0;
            }
        }
        else
        {
            obstacle_temp.laser_ids.push_back(i);
            if (check_new_class(sensors_info, j,i,0) )
            {
                new_flag=1;
                class_id++;
                obstacle_temp.n_lasers=obstacle_temp.laser_ids.size();
                v_obstacle_tmp.push_back(obstacle_temp);
            }
        }
    }
    obstacle_temp.laser_ids.push_back(60-30);
    obstacle_temp.n_lasers=obstacle_temp.laser_ids.size();       
    v_obstacle_tmp.push_back(obstacle_temp);
}

//深度相机的模拟激光数据用
double cal_obstacle_xxx_din(const sensor_msgs::LaserScan& laser_tmp,int temp_start_id, int temp_end_id,Laser_info laser_info)
{
    double x,y,x2,y2;
    y = laser_tmp.ranges[temp_start_id+180]* sin(temp_start_id*laser_info.angle_increment);
    x = laser_tmp.ranges[temp_start_id+180]* cos(temp_start_id*laser_info.angle_increment);
    y2 = laser_tmp.ranges[temp_end_id+180]* sin(temp_end_id*laser_info.angle_increment);
    x2 = laser_tmp.ranges[temp_end_id+180]* cos(temp_end_id*laser_info.angle_increment);

    return square_sum( x, x2, y, y2);
}

//深度相机的模拟激光数据用
void calculating_statistics_xxx(Laser_info laser_info,const sensor_msgs::LaserScan& laser_tmp,Obstacle &obstacle_temp)
{
    double temp_angular=0;
    int  temp_start_id=0;
    int  temp_end_id=0;
    temp_start_id= obstacle_temp.start_id;
    temp_end_id=   obstacle_temp.laser_ids[obstacle_temp.n_lasers-1];
    if (laser_tmp.ranges[obstacle_temp.laser_ids[0]+180]==laser_info.C_ranges_max_set)
    {  
        obstacle_temp.dmax=laser_info.C_ranges_max_set;
        obstacle_temp.dmin=laser_info.C_ranges_max_set;
        obstacle_temp.din2=cal_obstacle_xxx_din(laser_tmp,temp_start_id-1,temp_end_id+1, laser_info);
        obstacle_temp.dave= (laser_tmp.ranges[temp_start_id-1]+laser_tmp.ranges[temp_end_id+1])/2;
        obstacle_temp.pri_A=cal_pria(obstacle_temp,temp_start_id-1,temp_end_id+1, laser_info);

    }
    else
    {
        obstacle_temp.dmax=*max_element( laser_tmp.ranges.begin()+obstacle_temp.laser_ids[0], laser_tmp.ranges.begin()+obstacle_temp.laser_ids[0]+obstacle_temp.n_lasers);  
        obstacle_temp.dmin=*min_element( laser_tmp.ranges.begin()+obstacle_temp.laser_ids[0], laser_tmp.ranges.begin()+obstacle_temp.laser_ids[0]+obstacle_temp.n_lasers);
        obstacle_temp.din2=cal_obstacle_xxx_din(laser_tmp,temp_start_id,temp_end_id, laser_info);
        obstacle_temp.dave= (obstacle_temp.dmax+obstacle_temp.dmin)/2;
        if(laser_tmp.ranges[temp_start_id-1]==laser_info.C_ranges_max_set || laser_tmp.ranges[temp_end_id+1]==laser_info.C_ranges_max_set)
             obstacle_temp.pri_A=cal_pria(obstacle_temp,temp_start_id,temp_end_id, laser_info);
        else
            obstacle_temp.pri_A=cal_pria(obstacle_temp,temp_start_id-1,temp_end_id+1, laser_info);
    }
    obstacle_temp.angular=cal_Mid_Angle(obstacle_temp,laser_tmp)*D_180_div_PI;  // 单位为 度，不是弧度
    
}

//深度相机的模拟激光数据用
void nnc_fun_orbbec_laser(SensorsInfo sensors_info,const sensor_msgs::LaserScan& msg,vector<Obstacle>& v_obstacle_tmp)
{
    sensor_msgs::LaserScan laser_tmp;
    Car_info car_info=sensors_info.car_info;
    Laser_info laser_info=sensors_info.laser_info_orbbec;
    double y=0;
    double y2=0;
    double x=0;
    double x2=0;
    laser_tmp = msg;
    if (v_obstacle_tmp.size()!=0 || v_obstacle_tmp.capacity()!=0)
        vector<Obstacle>().swap(v_obstacle_tmp);
   // 最近邻聚类 
    nnc_xxx_laser(v_obstacle_tmp,laser_tmp, sensors_info);  

    // 计算统计量
    int  temp_start_id, temp_end_id;
    double temp_angular;
    if (v_obstacle_tmp.size()>2)
        for(int i=1; i<v_obstacle_tmp.size()-1; i++ )            
            calculating_statistics_xxx(laser_info,laser_tmp, v_obstacle_tmp[i]);
    //第一个类
    temp_start_id= v_obstacle_tmp[0].start_id;
    temp_end_id=   v_obstacle_tmp[0].laser_ids[v_obstacle_tmp[0].n_lasers-1];
    if (laser_tmp.ranges[v_obstacle_tmp[0].laser_ids[0]+180]==laser_info.C_ranges_max_set)
        {  
        
        v_obstacle_tmp[0].dmax=laser_info.C_ranges_max_set;
        v_obstacle_tmp[0].dmin=laser_info.C_ranges_max_set;
        v_obstacle_tmp[0].dave= laser_tmp.ranges[temp_end_id+1+180];
        v_obstacle_tmp[0].pri_A=laser_tmp.ranges[temp_end_id+1+180]* sin(laser_info.angle_increment)*(v_obstacle_tmp[0].laser_ids.size()+1);
        v_obstacle_tmp[0].din2=pow(v_obstacle_tmp[0].pri_A,2);

    }
    else
    {
        v_obstacle_tmp[0].dmax=*max_element( laser_tmp.ranges.begin()+v_obstacle_tmp[0].laser_ids[0]+180, laser_tmp.ranges.begin()+v_obstacle_tmp[0].laser_ids[0]+180+v_obstacle_tmp[0].n_lasers);  
        v_obstacle_tmp[0].dmin=*min_element( laser_tmp.ranges.begin()+v_obstacle_tmp[0].laser_ids[0]+180, laser_tmp.ranges.begin()+v_obstacle_tmp[0].laser_ids[0]+180+v_obstacle_tmp[0].n_lasers);
        y = laser_tmp.ranges[temp_start_id+180]* sin(temp_start_id*laser_info.angle_increment);
        x = laser_tmp.ranges[temp_start_id+180]* cos(temp_start_id*laser_info.angle_increment);
        y2 = laser_tmp.ranges[temp_end_id+180]* sin(temp_end_id*laser_info.angle_increment);
        x2 = laser_tmp.ranges[temp_end_id+180]* cos(temp_end_id*laser_info.angle_increment);
        v_obstacle_tmp[0].din2=pow(x-x2,2) + pow(y-y2,2);
        v_obstacle_tmp[0].dave= (v_obstacle_tmp[0].dmax+v_obstacle_tmp[0].dmin)/2;
    }
    // 单位为 度，不是弧度
    v_obstacle_tmp[0].angular=cal_Mid_Angle(v_obstacle_tmp[0],laser_tmp)*D_180_div_PI;     
    if (v_obstacle_tmp.size()>1)
    {
        //最后一个类
        int  last_obstacle_id=v_obstacle_tmp.size()-1;
        temp_start_id= v_obstacle_tmp[last_obstacle_id].start_id;
        temp_end_id=   v_obstacle_tmp[last_obstacle_id].laser_ids[v_obstacle_tmp[last_obstacle_id].n_lasers-1];
        if (laser_tmp.ranges[v_obstacle_tmp[last_obstacle_id].laser_ids[0]+180]==laser_info.C_ranges_max_set)
            {  
            
            v_obstacle_tmp[last_obstacle_id].dmax=laser_info.C_ranges_max_set;
            v_obstacle_tmp[last_obstacle_id].dmin=laser_info.C_ranges_max_set;
            v_obstacle_tmp[last_obstacle_id].dave= laser_tmp.ranges[temp_start_id-1+180];
            v_obstacle_tmp[last_obstacle_id].pri_A=laser_tmp.ranges[temp_start_id-1+180]* sin(laser_info.angle_increment)*(v_obstacle_tmp[0].laser_ids.size()+1);
            v_obstacle_tmp[last_obstacle_id].din2=pow(v_obstacle_tmp[last_obstacle_id].pri_A,2);

        }
        else
        {
            v_obstacle_tmp[last_obstacle_id].dmax=*max_element( laser_tmp.ranges.begin()+v_obstacle_tmp[last_obstacle_id].laser_ids[0]+180, laser_tmp.ranges.begin()+v_obstacle_tmp[last_obstacle_id].laser_ids[0]+180+v_obstacle_tmp[last_obstacle_id].n_lasers);  
            v_obstacle_tmp[last_obstacle_id].dmin=*min_element( laser_tmp.ranges.begin()+v_obstacle_tmp[last_obstacle_id].laser_ids[0]+180, laser_tmp.ranges.begin()+v_obstacle_tmp[last_obstacle_id].laser_ids[0]+180+v_obstacle_tmp[last_obstacle_id].n_lasers);
            y = laser_tmp.ranges[temp_start_id+180]* sin(temp_start_id*laser_info.angle_increment);
            x = laser_tmp.ranges[temp_start_id+180]* cos(temp_start_id*laser_info.angle_increment);
            y2 = laser_tmp.ranges[temp_end_id+180]* sin(temp_end_id*laser_info.angle_increment);
            x2 = laser_tmp.ranges[temp_end_id+180]* cos(temp_end_id*laser_info.angle_increment);
            v_obstacle_tmp[last_obstacle_id].din2=pow(x-x2,2) + pow(y-y2,2);
            v_obstacle_tmp[last_obstacle_id].dave= (v_obstacle_tmp[last_obstacle_id].dmax+v_obstacle_tmp[last_obstacle_id].dmin)/2;
        }
        // 单位为 度，不是弧度
        v_obstacle_tmp[last_obstacle_id].angular=cal_Mid_Angle(v_obstacle_tmp[last_obstacle_id],laser_tmp)*D_180_div_PI; 
    } 
};
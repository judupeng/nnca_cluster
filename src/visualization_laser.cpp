# include "nnca_cluster/visualization_laser.h"
# include "nnca_cluster/nnc_fun.h"



void showArrowGoal( visualization_msgs::Marker &arrow_goal,Obstacle &obstacle_temp,const sensor_msgs::LaserScan& msg,Laser_info laser_info)
{
    double y,x;
    std::stringstream ss;
     double angle_tmp=obstacle_temp.angular*D_PI_div_1_div_180;
    ss <<ros::Time::now().nsec;
    arrow_goal.ns="arrow_goal"+ ss.str();
    arrow_goal.lifetime=ros::Duration(0.2);
    arrow_goal.header.frame_id=msg.header.frame_id;
    arrow_goal.header.stamp = ros::Time::now();
    arrow_goal.action = visualization_msgs::Marker::ADD;
    arrow_goal.pose.orientation.w = 1.0;
    arrow_goal.id =300;
    arrow_goal.type = visualization_msgs::Marker::ARROW;
    arrow_goal.scale.x = 0.05;
    arrow_goal.scale.y = 0.05;
    arrow_goal.color.r = 1.0;
    arrow_goal.color.g = 0.0;
    arrow_goal.color.b = 0.0;
    arrow_goal.color.a = 1.0;
    y = 0.5* sin(angle_tmp);
    x = 0.5* cos(angle_tmp);
    arrow_goal.points.resize(2);
    arrow_goal.points[0].x = 0.0f;
    arrow_goal.points[0].y = 0.0f;
    arrow_goal.points[0].z = 0.0f;
    arrow_goal.points[1].x = x;
    arrow_goal.points[1].y = y;
    arrow_goal.points[1].z = 0.0f;
};


void showArrowAead(visualization_msgs::Marker &arrow_head,const sensor_msgs::LaserScan& msg,double lock_angle,int n)
{
    double y,x;
    double angle_tmp=lock_angle*D_PI_div_1_div_180;
    std::stringstream ss;
    ss <<ros::Time::now().nsec;
    arrow_head.ns="arrow_head"+ ss.str();;
    arrow_head.lifetime=ros::Duration(0.2);
    arrow_head.header.frame_id=msg.header.frame_id;
    arrow_head.header.stamp = ros::Time::now();
    arrow_head.action = visualization_msgs::Marker::ADD;
    arrow_head.pose.orientation.w = 1.0;
    arrow_head.id =301+ros::Time::now().nsec;
    arrow_head.type = visualization_msgs::Marker::ARROW;
    arrow_head.scale.x = 0.05;
    arrow_head.scale.y = 0.05;
    arrow_head.points.resize(2);
    arrow_head.points[0].x = 0.0f;
    arrow_head.points[0].y = 0.0f;
    arrow_head.points[0].z = 0.0f;
    if(n==0)
    {
        arrow_head.color.r = 0.0;
        arrow_head.color.g = 1.0;
        arrow_head.color.b = 0.0;
        arrow_head.color.a = 1.0;
        arrow_head.points[1].x = 0.5f;
        arrow_head.points[1].y = 0.0f;
        arrow_head.points[1].z = 0.0f;
    }
    else if (n==1)
    {
        arrow_head.color.r = 0.0;
        arrow_head.color.g = 0.0;
        arrow_head.color.b = 0.0;
        arrow_head.color.a = 0.6;
        arrow_head.points[1].x = 0.5f*cos(angle_tmp);
        arrow_head.points[1].y = -0.5f*sin(angle_tmp);
        arrow_head.points[1].z = 0.0f;
    }
    else if (n==2)
    {
        arrow_head.color.r = 0.0;
        arrow_head.color.g = 0.0;
        arrow_head.color.b = 0.0;
        arrow_head.color.a = 0.6;
        arrow_head.points[1].x = 0.5f*cos(angle_tmp);
        arrow_head.points[1].y = 0.5f*sin(angle_tmp);
        arrow_head.points[1].z = 0.0f;
    }
};


void showMergerLaser(const sensor_msgs::LaserScan& msg, ros::Publisher &scan_debug_marker_pub)
{
    sensor_msgs::LaserScan laser_tmp= msg;
    visualization_msgs::Marker points;
    visualization_msgs::MarkerArray line_list_Array_debug;
    std::stringstream ss;
    for (int id=0; id<4;id++)
    {
        ss <<ros::Time::now().nsec;
        points.ns = "points"+ ss.str();;
        points.header.frame_id=msg.header.frame_id;
        points.header.stamp = ros::Time::now();
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = id;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        points.color = createColorRGBA(int(id%4));

        for (int i=id*90; i<(id+1)*90;i++)
        {
        double y = laser_tmp.ranges[i]* sin(cal_Angle(i,laser_tmp));
        double x = laser_tmp.ranges[i]* cos(cal_Angle(i,laser_tmp));
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = 0;
        points.colors.push_back(points.color);
        points.points.push_back(p);    
        }
        line_list_Array_debug.markers.push_back(points);
        vector<geometry_msgs::Point>().swap(points.points);
        vector<std_msgs::ColorRGBA>().swap(points.colors);
    }
    scan_debug_marker_pub.publish(line_list_Array_debug);
    vector<visualization_msgs::Marker>().swap(line_list_Array_debug.markers);
};

void showPointsCluster(Obstacle &obstacle_temp,int class_id,const sensor_msgs::LaserScan& msg,
    visualization_msgs::Marker &line_list,visualization_msgs::Marker &text_view,Laser_info   laser_info)
{
    int end_id;
    double y,x;
    geometry_msgs::Point p;
    std::stringstream ss;
    ss <<ros::Time::now().nsec;
    if (line_list.colors.size()!=0 || line_list.colors.capacity()!=0)
        vector<std_msgs::ColorRGBA>().swap(line_list.colors);
    if (line_list.points.size()!=0 || line_list.points.capacity()!=0)
        vector<geometry_msgs::Point>().swap(line_list.points);

    line_list.ns ="line_list"+ ss.str();
    text_view.ns="text"+ ss.str();
    line_list.header.frame_id=text_view.header.frame_id=msg.header.frame_id;
    line_list.header.stamp =text_view.header.stamp = ros::Time::now();
    line_list.action = text_view.action =visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w =text_view.pose.orientation.w = 1.0;
    line_list.id = obstacle_temp.class_id;
    text_view.id =obstacle_temp.class_id+ 200;
    line_list.lifetime=ros::Duration(0.2);
    text_view.lifetime=ros::Duration(0.2);
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    text_view.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    
    if (obstacle_temp.class_id == class_id)
    {
        text_view.scale.z = 0.4;
        line_list.scale.x = 0.04;
        line_list.color = text_view.color = createColorRGBA(9);
    }
    else
    {
        text_view.scale.z = 0.2;
        line_list.scale.x = 0.02;
        line_list.color = text_view.color = createColorRGBA(int(obstacle_temp.class_id%9));
    }
    text_view.text=to_string(int(obstacle_temp.pri))+ ":" +to_string(int(obstacle_temp.class_id));
    end_id= obstacle_temp.laser_ids[obstacle_temp.n_lasers-1];

    if (obstacle_temp.start_id <= end_id )
        for (int i=obstacle_temp.start_id; i<obstacle_temp.n_lasers+obstacle_temp.start_id;i++)
        {
            y = msg.ranges[i]* sin(cal_Angle(i,msg));
            x = msg.ranges[i]* cos(cal_Angle(i,msg));
            
            p.x = x;
            p.y = y;
            p.z = 0;
            line_list.colors.push_back(line_list.color);
            line_list.points.push_back(p);
            p.y += 0.02;
            line_list.colors.push_back(line_list.color);
            line_list.points.push_back(p);     
        }
    else
        {
            for (int i=obstacle_temp.start_id; i<laser_info.LASRE_BEAM;i++)
            {
                y = msg.ranges[i]* sin(cal_Angle(i,msg));
                x = msg.ranges[i]* cos(cal_Angle(i,msg));
                p.x = x;
                p.y = y;
                p.z = 0;
                line_list.colors.push_back(line_list.color);
                line_list.points.push_back(p);
                p.y += 0.02;
                line_list.colors.push_back(line_list.color);
                line_list.points.push_back(p);     
            }
            for (int i=0; i<end_id+1;i++)
            {
                y = msg.ranges[i]* sin(cal_Angle(i,msg));
                x = msg.ranges[i]* cos(cal_Angle(i,msg));
                p.x = x;
                p.y = y;
                p.z = 0;
                line_list.colors.push_back(line_list.color);
                line_list.points.push_back(p);
                p.y += 0.02;
                line_list.colors.push_back(line_list.color);
                line_list.points.push_back(p);     
            }
        }
    y = msg.ranges[obstacle_temp.start_id]* sin(cal_Angle(obstacle_temp.start_id,msg));
    x = msg.ranges[obstacle_temp.start_id]* cos(cal_Angle(obstacle_temp.start_id,msg));
    p.x = x;
    p.y = y;
    y = msg.ranges[end_id]* sin(cal_Angle(end_id,msg));
    x = msg.ranges[end_id]* cos(cal_Angle(end_id,msg));
    p.x= (p.x+x)/2;
    p.y= (p.y+y)/2;
    if (p.x>=0 && p.y>=0)
    {
        p.x=p.x+0.2;
        p.y=p.y+0.2;
    }
    else if(p.x>=0 && p.y<0)
    {
        p.x=p.x+0.2;
        p.y=p.y-0.2;
    }
    else if(p.x<0 && p.y>=0)
    {
        p.x=p.x-0.2;
        p.y=p.y+0.2;
    }
    else if(p.x<0 && p.y<0)
    {
        p.x=p.x-0.2;
        p.y=p.y-0.2;
    }
    p.z = 0;
    text_view.pose.position=p;
};




void showPointsPlusterOrbbec(Obstacle &obstacle_temp,const sensor_msgs::LaserScan& msg,
    visualization_msgs::Marker &line_list_orbbec,visualization_msgs::Marker &text_view_orbbec,Laser_info   laser_info)
{

    int end_id;
    double y,x;
    geometry_msgs::Point p;
    std::stringstream ss;
    ss <<ros::Time::now().nsec;
    if (line_list_orbbec.colors.size()!=0 || line_list_orbbec.colors.capacity()!=0)
        vector<std_msgs::ColorRGBA>().swap(line_list_orbbec.colors);
    if (line_list_orbbec.points.size()!=0 || line_list_orbbec.points.capacity()!=0)
        vector<geometry_msgs::Point>().swap(line_list_orbbec.points);

    line_list_orbbec.ns ="line_list_orbbec"+ ss.str();
    text_view_orbbec.ns="text_orbbec"+ ss.str();
    line_list_orbbec.header.frame_id=text_view_orbbec.header.frame_id=msg.header.frame_id;
    line_list_orbbec.header.stamp =text_view_orbbec.header.stamp =ros::Time::now();
    line_list_orbbec.action = text_view_orbbec.action =visualization_msgs::Marker::ADD;
    line_list_orbbec.pose.orientation.w =text_view_orbbec.pose.orientation.w = 1.0;
    line_list_orbbec.id = obstacle_temp.class_id;
    text_view_orbbec.id =obstacle_temp.class_id+ 200;
    line_list_orbbec.lifetime=ros::Duration(0.2);
    text_view_orbbec.lifetime=ros::Duration(0.2);
    line_list_orbbec.type = visualization_msgs::Marker::LINE_LIST;
    text_view_orbbec.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    

    text_view_orbbec.scale.z = 0.2;
    text_view_orbbec.text=to_string(int(obstacle_temp.pri))+ ":" +to_string(int(obstacle_temp.class_id));
    line_list_orbbec.scale.x = 0.02;
    line_list_orbbec.color = text_view_orbbec.color = createColorRGBA(int(obstacle_temp.class_id%9));
    end_id= obstacle_temp.laser_ids[obstacle_temp.n_lasers-1];

    if (obstacle_temp.start_id <= end_id )
        for (int i=obstacle_temp.start_id; i<obstacle_temp.n_lasers+obstacle_temp.start_id;i++)
        {
            y = msg.ranges[i+180]* sin(i*laser_info.angle_increment);
            x = msg.ranges[i+180]* cos(i*laser_info.angle_increment);
            
            p.x = x;
            p.y = y;
            p.z = 0;
            line_list_orbbec.colors.push_back(line_list_orbbec.color);
            line_list_orbbec.points.push_back(p);
            p.y += 0.02;
            line_list_orbbec.colors.push_back(line_list_orbbec.color);
            line_list_orbbec.points.push_back(p);     
        }
    else
        {
            for (int i=obstacle_temp.start_id; i<laser_info.LASRE_BEAM;i++)
            {
                y = msg.ranges[i+180]* sin(i*laser_info.angle_increment);
                x = msg.ranges[i+180]* cos(i*laser_info.angle_increment);
                p.x = x;
                p.y = y;
                p.z = 0;
                line_list_orbbec.colors.push_back(line_list_orbbec.color);
                line_list_orbbec.points.push_back(p);
                p.y += 0.02;
                line_list_orbbec.colors.push_back(line_list_orbbec.color);
                line_list_orbbec.points.push_back(p);     
            }
            for (int i=0; i<end_id+1;i++)
            {
                y = msg.ranges[i+180]* sin(i*laser_info.angle_increment);
                x = msg.ranges[i+180]* cos(i*laser_info.angle_increment);
                p.x = x;
                p.y = y;
                p.z = 0;
                line_list_orbbec.colors.push_back(line_list_orbbec.color);
                line_list_orbbec.points.push_back(p);
                p.y += 0.02;
                line_list_orbbec.colors.push_back(line_list_orbbec.color);
                line_list_orbbec.points.push_back(p);     
            }
        }
    y = msg.ranges[obstacle_temp.start_id+180]* sin(obstacle_temp.start_id*laser_info.angle_increment);
    x = msg.ranges[obstacle_temp.start_id+180]* cos(obstacle_temp.start_id*laser_info.angle_increment);
    p.x = x;
    p.y = y;
    y = msg.ranges[end_id+180]* sin(end_id*laser_info.angle_increment);
    x = msg.ranges[end_id+180]* cos(end_id*laser_info.angle_increment);
    p.x= (p.x+x)/2;
    p.y= (p.y+y)/2;
    if (p.x>=0 && p.y>=0)
    {
        p.x=p.x+0.2;
        p.y=p.y+0.2;
    }
    else if(p.x>=0 && p.y<0)
    {
        p.x=p.x+0.2;
        p.y=p.y-0.2;
    }
    else if(p.x<0 && p.y>=0)
    {
        p.x=p.x-0.2;
        p.y=p.y+0.2;
    }
    else if(p.x<0 && p.y<0)
    {
        p.x=p.x-0.2;
        p.y=p.y-0.2;
    }
    p.z = 0;
    text_view_orbbec.pose.position=p;
};



void showDepthPointsProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_Y_cloud_ptr,const sensor_msgs::PointCloud2ConstPtr& msg,
    ros::Publisher &depth_projection_markers_pub)
{
    int end_id;
    double y,x,z;
    geometry_msgs::Point p;
    std::stringstream ss;
    ss <<ros::Time::now().nsec;
    visualization_msgs::Marker line_list_depth_projection;
    if (line_list_depth_projection.colors.size()!=0 || line_list_depth_projection.colors.capacity()!=0)
        vector<std_msgs::ColorRGBA>().swap(line_list_depth_projection.colors);
    if (line_list_depth_projection.points.size()!=0 || line_list_depth_projection.points.capacity()!=0)
        vector<geometry_msgs::Point>().swap(line_list_depth_projection.points);

    line_list_depth_projection.ns ="line_list_depth_projection"+ss.str();
    line_list_depth_projection.header.frame_id=msg->header.frame_id;
    line_list_depth_projection.header.stamp =ros::Time::now();
    line_list_depth_projection.action =visualization_msgs::Marker::ADD;
    line_list_depth_projection.pose.orientation.w = 1.0;
    line_list_depth_projection.id =1000;
    //line_list_depth_projection.lifetime=ros::Duration(0.2);
    line_list_depth_projection.type = visualization_msgs::Marker::LINE_LIST;
    line_list_depth_projection.scale.x = 0.02;
    line_list_depth_projection.color =createColorRGBA(9);

    for (int i=0; i<clipped_Y_cloud_ptr->points.size();i++)
    {
        z = clipped_Y_cloud_ptr->points[i].z;
        x = clipped_Y_cloud_ptr->points[i].x;
        p.x = x;
        p.y = 0;
        p.z = z;
        line_list_depth_projection.colors.push_back(line_list_depth_projection.color);
        line_list_depth_projection.points.push_back(p);
        p.y += 0.02;
        line_list_depth_projection.colors.push_back(line_list_depth_projection.color);
        line_list_depth_projection.points.push_back(p);    
    }
    depth_projection_markers_pub.publish(line_list_depth_projection); 
    vector<std_msgs::ColorRGBA>().swap(line_list_depth_projection.colors);
    vector<geometry_msgs::Point>().swap(line_list_depth_projection.points);
};
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include "nnca_cluster/nnca_cluster_node.h"
#include "nnca_cluster/priority_cal.h"
#include "nnca_cluster/nnc_fun.h"
#include "decision/memory_route.h"

using namespace std;
using namespace ros;
using namespace tf;
namespace nnca
{
NncaClusterNode::NncaClusterNode(const ros::NodeHandle& nh_, 
                           const ros::NodeHandle& nh_private_)
: 
  nh(nh_),
  nh_private(nh_private_),
  last_cmd_angular_speed_orbbec(0),
  last_cmd_linear_speed_orbbec(0),
  last_cmd_angular_speed(0),
  last_cmd_linear_speed(0),
  fusion_method(1), //等于0松耦合先做决策后进行决策投票，暂未实现不可用；等于1紧耦合先融合数据后做一次决策。
  is_orbbec_scan(0),
  is_orbbec_only(0),
  is_current_direction(0),
  is_global_direction(0),
  is_First(1),
  is_merger_laser(0),
  is_freeze_Local(0),
  is_freeze_Global(0),
  is_orbbec_First(1),
  is_orbbec_freeze_Local(1),
  mag_flag(0),
  infrared_flag(0),
  ultrasound_flag(0),
  is_show(1),
  is_current_gps_pose(0),
  n_runState(0),
  STATE(INIT),
  MOVE_STATE(STOP_AND_WAIT)
{

    initializeParams();
    tf::Quaternion quaternion;
	tf::Transform transform;
    last_orbbec_laser_time=ros::Time::now();
    last_laser_time=ros::Time::now();
    ros::Rate loop_rate(30);
    std::string log_folder = "/tmp";
    SAVE_MOVE_PATH_FILE_.open(log_folder+"/save_move_path.txt");
    SAVE_PLAN_PATH_FILE_.open(log_folder+"/save_plan_path.txt");
    //double roll = -0.02, pitch = 0.12, yaw = 0.12;
    double roll = 0, pitch = 0, yaw = 0;
    double tx = 0.1, ty = 0.04, tz = 0.09;
    twist_cmd=geometry_msgs::TwistStamped();
    twist_cmd_last=geometry_msgs::TwistStamped();
	quaternion.setRPY(roll, pitch, yaw);
    transform.setOrigin(tf::Vector3(tx, ty, tz));
	transform.setRotation(quaternion);
    ros::Subscriber depth_points_sub = nh.subscribe("/camera/depth/points", 1, &NncaClusterNode::depthPointsCallback,this);
    ros::Subscriber laser_sub = nh.subscribe("/T4_scan", 1000, &NncaClusterNode::laserCallback,this); /// /T4_scan  /scan
    ros::Subscriber orbbec_laser_sub = nh.subscribe("/orbbec/scan", 1, &NncaClusterNode::orbbecLaserCallback,this);
    ros::Subscriber merger_laser_sub = nh.subscribe("/merger_scan", 1, &NncaClusterNode::mergerLaserCallback,this);
    ros::Subscriber ultrasound_sub = nh.subscribe("/ultrasound_range", 1, &NncaClusterNode::ultrasoundCallback,this);
    ros::Subscriber infrared_sub = nh.subscribe("/infrared_range", 1, &NncaClusterNode::infraredCallback,this);
    ros::Subscriber ekf_odom_combined_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 1, &NncaClusterNode::ekfOdomCombinedCallback,this);
    ros::Subscriber gps_route_cmd_sub = nh.subscribe("/route_cmd", 10, &NncaClusterNode::gpsRouteCmdCallback,this);
    //ros::Subscriber imu_mag_sub=nh.subscribe("/imu/mag",1,&NncaClusterNode::imuMagCallback,this);
    //ros::Subscriber racm1_gps_sub = nh.subscribe("/racm1_gnss_pose", 5, &NncaClusterNode::racm1GpsCallback,this);
    ros::Subscriber yaw_with_north_sub = nh.subscribe("/yaw", 2, &NncaClusterNode::yawWithNorthCallback,this);
    ros::Subscriber gnss_pose_sub=nh.subscribe("/gnss_pose_comb",2,&NncaClusterNode::gnssPoseCallback,this);
    scan_cluter_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("scan_cluter_markers", 1);
    depth_projection_markers_pub = nh.advertise<visualization_msgs::Marker>("depth_projection_markers", 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_raw", 1);
    point_z_filter_pub=  nh.advertise<sensor_msgs::PointCloud2>("/point_z_filter", 1);
    mag_markers_pub = nh.advertise<visualization_msgs::Marker>("imu_mag_marker", 1);
    path_map_info.path_odom_combined_pub = nh.advertise<nav_msgs::Path>("trajectory_odom_combined",1, true);
    path_map_info.plan_path_on_map_pub = nh.advertise<nav_msgs::Path>("plan_path_on_map",1, true);
    path_map_info.plan_path_on_map_dynamic_pub=nh.advertise<nav_msgs::Path>("plan_path_on_map_dynamic",1, true);
    path_map_info.move_path_on_map_pub = nh.advertise<nav_msgs::Path>("move_path_on_map",1, true);
    //bezier_controlpoint_path_pub=nh.advertise<nav_msgs::Path>("bezier_controlpoint",1, true);
    //bezier_curve_path_pub=nh.advertise<nav_msgs::Path>("bezier_curve",1, true);
    //turn_goal_angle(5,0.1);
    boost::thread thread_nnca_cluster(boost::bind(&NncaClusterNode::processCluster,this));
    while (ros::ok())
    {    
        //br.sendTransform(tf::StampedTransform(
            //transform, ros::Time::now(), "base_scan", "camera_link"));
        ////br.sendTransform(tf::StampedTransform(
            ////tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.06, 0.025, 0.09)),
        ////ros::Time::now(),"base_link", "base_laser"));
        ros::spinOnce();
        runCmd(twist_cmd,twist_cmd_last);
        loop_rate.sleep();
    }
};

NncaClusterNode::~NncaClusterNode(){
SAVE_MOVE_PATH_FILE_.close();
SAVE_PLAN_PATH_FILE_.close();
};

void NncaClusterNode:: initializeParams()
{
    nh_private.param<double>("car_width", car_info.WIDTH , 0.14);
    nh_private.param<double>("car_long", car_info.LONG , 0.14);
    nh_private.param<double>("max_vel_x", car_info.max_vel_x , 0.3);
    nh_private.param<double>("min_vel_x", car_info.min_vel_x , -0.3);
    nh_private.param<double>("max_turn_x", car_info.max_turn_x , 1.2);
    nh_private.param<double>("acc_lim_x", car_info.acc_lim_x , 4);
    nh_private.param<double>("acc_lim_th", car_info.acc_lim_th , 4);
    nh_private.param<double>("turn_factor", car_info.turn_factor , 1.2);
    nh_private.param<double>("vel_factor", car_info.vel_factor , 1.2);
    nh_private.param<double>("wheel_interval", car_info.wheel_interval , 0.126);
    nh_private.param<double>("wheel_diameter", car_info.wheel_diameter , 0.065);
    nh_private.param<double>("car_long", car_info.LONG , 0.14);
    nh_private.param<int>("orbbec_LASRE_BEAM", laser_info_orbbec.LASRE_BEAM , 60);
    nh_private.param<double>("lock_angle", obstacle_info.lock_angle , 75);
    nh_private.param<double>("turning_angle", obstacle_info.turning_angle , 30);
    nh_private.param<double>("orbbec_turning_angle", obstacle_info_orbbec.turning_angle , 15);
    nh_private.param<double>("mid_distance", obstacle_info.mid_distance , 0.5);
    nh_private.param<double>("left_distance", obstacle_info.left_distance , 0.3);
    //从launch读取模糊控制文件用
    std::string OA_fis_file_;
    std::string GO_fis_file_;
    nh_private.getParam("OA_fis_file", OA_fis_file_ );
    nh_private.getParam("GO_fis_file", GO_fis_file_ );
    int len = OA_fis_file_.length();
    OA_fis_file=new char[len+1];
    strcpy(OA_fis_file,OA_fis_file_.c_str());
    len = GO_fis_file_.length();
    GO_fis_file=new char[len+1];
    strcpy(GO_fis_file,GO_fis_file_.c_str());
    //不从launch读取模糊控制文件用
    OA_fis_file="/home/one/catkin_ws/src/nnca_cluster/obstacle_avoidance.fis";
    GO_fis_file="/home/one/catkin_ws/src/nnca_cluster/goal_oriented4.fis";
    //激光雷达参数，聚类用参数
    laser_info.C_LASRE_THRESHOLD= 7.5;  //4 5 6  6.5  7
    laser_info.C=7.5;  //4 5 6  6.5  7
    laser_info_orbbec.C_LASRE_THRESHOLD= 7.5;  //4 5 6  6.5  7
    laser_info_orbbec.C=7.5;  //4 5 6  6.5  7
    //小车信息参数
    car_info.C_BODY = 0.14;
    //障碍物参数
    obstacle_info.n_sign_cunt=0;
    obstacle_info.max_pri=-1;
    obstacle_info.sel_max_pri_id=-1;
    obstacle_info.real_distance=1000;
    obstacle_info.real_angle=1000;
    //障碍物参数，深度相机用
    obstacle_info_orbbec.n_sign_cunt=0;
    obstacle_info_orbbec.max_pri=-1;
    obstacle_info_orbbec.sel_max_pri_id=-1;
    obstacle_info_orbbec.real_distance=1000;
    obstacle_info_orbbec.real_angle=1000;
    //小车信息参数 +  激光雷达参数
    sensor_info.car_info=car_info;
    sensor_info.laser_info=laser_info;
    sensor_info.laser_info_orbbec=laser_info_orbbec;

    double WIDTH=car_info.WIDTH;
    // GPS、路径、地图信息参数
    path_map_info.current_direction=0;
    path_map_info.global_direction=0;
    path_map_info.yaw_with_north=0;
    path_map_info.yaw_with_north_globle=0;
    // 膨胀用参数，暂时不用了
    inflate_info.inflate_l_r=WIDTH *1.1;
    inflate_info.inflate_front=WIDTH *2.5;
    inflate_info.inflate_back=0;
    inflate_info.left_top_lsaer_id= int(atan2(inflate_info.inflate_l_r,inflate_info.inflate_front)*D_180_div_PI);
    inflate_info.right_top_lsaer_id= 360-inflate_info.left_top_lsaer_id;
    inflate_info.left_lsaer_id= 90;
    inflate_info.right_lsaer_id= 270;

    FIS_FILES.push_back(OA_fis_file);
    FIS_FILES.push_back(GO_fis_file);
   
    double threshold_arr_temp[23]={WIDTH,1.5*WIDTH,2*WIDTH,2.5*WIDTH,3*WIDTH,4*WIDTH,5*WIDTH
            ,6*WIDTH,7*WIDTH,8*WIDTH,9*WIDTH,10*WIDTH,11*WIDTH,12*WIDTH,13*WIDTH
            ,14*WIDTH,15*WIDTH,15*WIDTH,17*WIDTH,18*WIDTH,19*WIDTH,20*WIDTH,21*WIDTH};
    for (int i=0; i< 23;i++)
        threshold_arr.push_back(threshold_arr_temp[i]);
}

void NncaClusterNode:: gpsRouteCmdCallback(const tablet_socket_msgs::route_cmd& msg)
{
    ROS_INFO("gpsRouteCmdCallback");
    geometry_msgs::PoseStamped pose_tmp;
    std::vector<Point3> route_xyz;
    getRouteXYZ(msg,&route_xyz);
    vector<geometry_msgs::PoseStamped>().swap(path_map_info.plan_path_on_map.poses); 

    for(int i=0; i<route_xyz.size();i++)
    {
        pose_tmp.header.frame_id="map";
        pose_tmp.header.stamp=msg.header.stamp;
        pose_tmp.pose.position.x=route_xyz[i].x;
        pose_tmp.pose.position.y=route_xyz[i].y;
        path_map_info.plan_path_on_map.poses.push_back(pose_tmp);
        SAVE_PLAN_PATH_FILE_ << std::fixed << std::setprecision(15)<< route_xyz[i].x << "," <<route_xyz[i].y << "\n";
        //save plan point(x,y) to /tmp/save_plan_path.txt
    }
    path_map_info.plan_path_on_map.header.frame_id="map";
    path_map_info.plan_path_on_map_pub.publish(path_map_info.plan_path_on_map);  //plan_path_on_map保存
    is_global_direction=1;
    std::vector<double> distance;
    getPointDistance(route_xyz,&distance);
	for(int i=0;i<distance.size();i++)
    { std::cout<<"第"<<i+1<<"段："<<distance[i]<<std::endl; }
}
void NncaClusterNode:: gnssPoseCallback(const geometry_msgs::PoseStamped& pose_msg)
{
    /*ros::Duration transform_tolerance_(0.2);
	ros::Time now= (pose_msg.header.stamp + transform_tolerance_);
    tf::TransformBroadcaster br2;
    tf::Transform transform2;
    tf::Quaternion q2;
    tf::quaternionMsgToTF(pose_msg.pose.orientation, q2);   
    transform2.setOrigin(tf::Vector3(pose_msg.pose.position.x, pose_msg.pose.position.y,0));
    transform2.setRotation(q2);
    br2.sendTransform(tf::StampedTransform(transform2,now,"map", "base_footprint"));*/
    path_map_info.current_gps_pose=pose_msg;
    path_map_info.current_gps_pose.header.frame_id="map";
    SAVE_MOVE_PATH_FILE_ << std::fixed << std::setprecision(15)<< path_map_info.current_gps_pose.pose.position.x << "," \
                            << path_map_info.current_gps_pose.pose.position.y << "\n";
    //save move point(x,y) to /tmp/save_move_path.txt
    if (path_map_info.move_path_on_map.poses.size()>0)
        if(path_map_info.move_path_on_map.poses.back().pose.position.x!= path_map_info.current_gps_pose.pose.position.x && \
            path_map_info.move_path_on_map.poses.back().pose.position.y!= path_map_info.current_gps_pose.pose.position.y)
            path_map_info.move_path_on_map.poses.push_back(path_map_info.current_gps_pose);
    else
        path_map_info.move_path_on_map.poses.push_back(path_map_info.current_gps_pose);

    path_map_info.move_path_on_map.header.frame_id="map";
    path_map_info.move_path_on_map_pub.publish(path_map_info.move_path_on_map);//move_path_on_map保存
    is_current_gps_pose=1;
}

/*void NncaClusterNode:: racm1GpsCallback(const sensor_msgs::NavSatFix msg)
{
    fix_msg=msg;
    Point3 p_xyz = lla_xyz(msg.latitude,msg.longitude,0.0);
    tf::TransformBroadcaster br_tmp;
    tf::Quaternion quaternion_tmp;
	tf::Transform transform_mag_tmp;
    ros::Duration transform_tolerance_(0.2);
	ros::Time now= (msg.header.stamp + transform_tolerance_);
    double roll = 0, pitch = 0, yaw = 0;
    double tx = p_xyz.x, ty = p_xyz.y, tz = 0;
	quaternion_tmp.setRPY(roll, pitch, yaw);
    transform_mag_tmp.setOrigin(tf::Vector3(tx, ty, tz));
	transform_mag_tmp.setRotation(quaternion_tmp);
    br_tmp.sendTransform(tf::StampedTransform(\
            transform_mag_tmp, ros::Time::now(), "/map","/base_footprint"));
    //br_tmp.sendTransform(tf::StampedTransform(\
            transform_mag_tmp, ros::Time::now(), "/map","/odom"));        
}*/

void NncaClusterNode:: imuMagCallback(const sensor_msgs::MagneticField& mag_msg_tmp)
{
    imu_mag_msg=mag_msg_tmp;
    mag_flag=1;
    double xx= 0,yy= 0,zz= 0,angle= D_PI_div_2;
    xx=pow(imu_mag_msg.magnetic_field.x,2);
    yy=pow(imu_mag_msg.magnetic_field.y,2);
    zz=pow(imu_mag_msg.magnetic_field.z,2);
    if (imu_mag_msg.magnetic_field.z<= 0)
        angle=-acos((xx+yy)/sqrt((xx+yy)*(xx+yy+zz)))*D_180_div_PI;
    else
        angle=acos((xx+yy)/sqrt((xx+yy)*(xx+yy+zz)))*D_180_div_PI;
    path_map_info.current_direction=atan2(imu_mag_msg.magnetic_field.x,imu_mag_msg.magnetic_field.y)*D_180_div_PI;
    is_current_direction=1;
}

void NncaClusterNode:: yawWithNorthCallback(const std_msgs::Float64 msg)
{
    path_map_info.yaw_with_north=msg.data*D_180_div_PI;
    is_current_direction=1;
}

void NncaClusterNode:: ultrasoundCallback(const sensor_msgs::LaserScan& msg)
{
    ultrasound_msg=msg;
    ultrasound_flag=1;
};

void NncaClusterNode:: infraredCallback(const sensor_msgs::LaserScan& msg)
{
     infrared_msg=msg;
     infrared_flag=1;
};

void NncaClusterNode:: mergerLaserCallback(const sensor_msgs::LaserScan& msg)
{
    merger_scan_msg=msg;
    //showMergerLaser(merger_scan_msg);
    is_merger_laser=1;
};

void NncaClusterNode:: depthPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_Y_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::fromROSMsg(*msg, *current_sensor_cloud_ptr);
    pass.setInputCloud (current_sensor_cloud_ptr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits ( -0.2, 0.1);
    pass.filter (*clipped_Y_cloud_ptr);
    pcl::toROSMsg(*clipped_Y_cloud_ptr,cloud_msg);
    cloud_msg.header.frame_id = msg->header.frame_id;
    cloud_msg.header.stamp= ros::Time::now();
    point_z_filter_pub.publish(cloud_msg);
    showDepthPointsProjection(clipped_Y_cloud_ptr,msg,depth_projection_markers_pub);
};

void NncaClusterNode:: runCmd(geometry_msgs::TwistStamped &_twist_cmd,geometry_msgs::TwistStamped &_twist_cmd_last)
{
    cmd_vel_pub.publish(_twist_cmd);
    _twist_cmd_last=_twist_cmd;
};

void NncaClusterNode:: ekfOdomCombinedCallback(const geometry_msgs::PoseWithCovarianceStamped &ekf_Pose_msg)
{
    path_map_info.pose_Stamped.header=ekf_Pose_msg.header;
    path_map_info.pose_Stamped.header.frame_id="/odom";
    path_map_info.pose_Stamped.pose=ekf_Pose_msg.pose.pose;
    path_map_info.odom_path.header.stamp =path_map_info.pose_Stamped.header.stamp;
    path_map_info.odom_path.header.frame_id ="/odom";
    path_map_info.odom_path.poses.push_back(path_map_info.pose_Stamped);
    path_map_info.path_odom_combined_pub.publish(path_map_info.odom_path);
};

void NncaClusterNode:: orbbecLaserCallback(const sensor_msgs::LaserScan& input)   //深度相机的模拟激光数据用
{
    Obstacle sel_max_pri_Obstacle;
    orbbec_scan_msg=input;
    is_orbbec_scan=1;
    //聚类   
    laser_info_orbbec.laser_begin_id=0;
    laser_info_orbbec.angle_increment=input.angle_increment;
    if (orbbec_scan_msg.header.stamp.toSec()-last_laser_time.toSec()>5)
    {
        is_orbbec_only=1;
        nnc_fun_orbbec_laser(sensor_info,input,obstacle_info_orbbec.v_obstacle);
        orbbecPrioritySet(obstacle_info_orbbec);
        sel_max_pri_Obstacle=orbbecCmdSet(obstacle_info_orbbec,twist_cmd);
        if (is_show==1)
        {
            for (int i=0; i<obstacle_info_orbbec.v_obstacle.size() ;i++)
            {
                showPointsPlusterOrbbec(obstacle_info_orbbec.v_obstacle[i],orbbec_scan_msg,line_list_orbbec,text_view_orbbec,laser_info_orbbec);
                line_list_Array_orbbec.markers.push_back(line_list_orbbec);
                line_list_Array_orbbec.markers.push_back(text_view_orbbec);
                vector<std_msgs::ColorRGBA>().swap(line_list_orbbec.colors);
                vector<geometry_msgs::Point>().swap(line_list_orbbec.points);
            }
            showArrowGoal(arrow_goal,sel_max_pri_Obstacle,orbbec_scan_msg, laser_info);
            line_list_Array.markers.push_back(arrow_goal);
            showArrowAead(arrow_head,orbbec_scan_msg,obstacle_info_orbbec.lock_angle,0);
            line_list_Array.markers.push_back(arrow_head);
            showArrowAead(arrow_head,orbbec_scan_msg,obstacle_info_orbbec.lock_angle,1);
            line_list_Array.markers.push_back(arrow_head);
            showArrowAead(arrow_head,orbbec_scan_msg,obstacle_info_orbbec.lock_angle,2);
            line_list_Array.markers.push_back(arrow_head);
            scan_cluter_markers_pub.publish(line_list_Array_orbbec);
            vector<visualization_msgs::Marker>().swap(line_list_Array_orbbec.markers);
        }
    }
    else
        is_orbbec_only=0;
};


void NncaClusterNode::processCluster()
{
    Obstacle sel_max_pri_Obstacle;
    while(1)
    {
        if(scan_deque.size()>0)
        {
            scan_msg=scan_deque.front();
            scan_deque.pop_front();
            //聚类
            if (is_merger_laser==0 || fusion_method==0)  //仅激光雷达存在情况下分支                                      
            {
                pre_process(scan_msg,laser_info);//数据预处理
                sensor_info.laser_info=laser_info;
                nnc_fun(sensor_info,obstacle_info.v_obstacle);  //对激光雷达数据做环形聚类  和  计算聚类后每一类统计量
            }
            else //激光雷达和深度相机同时存在情况下分支
            {
                is_merger_laser=0;
                pre_process(scan_msg,laser_info);//数据预处理
                sensor_info.laser_info=laser_info;
                nnc_fun(sensor_info,obstacle_info.v_obstacle);  //对激光雷达数据做环形聚类
            }
            //设置规则、优先级
            laserPrioritySet(obstacle_info,obstacle_info_orbbec);  // 计算每一类的优先级大小写入统计数据
            //发布命令
            geometry_msgs::TwistStamped twist_cmd_tmp= geometry_msgs::TwistStamped();
            sel_max_pri_Obstacle=cmdSet(obstacle_info,path_map_info,twist_cmd);// 对计算后的优先级选择最好的优先级，然后根据
                                                                                //选择的优先级设置线速度和角速度：twist_cmd
            // 显示 sel_max_pri_Obstacle          
            if (is_show==1)       
            {
                for (int i=0; i<obstacle_info.v_obstacle.size() ;i++)
                {
                    showPointsCluster(obstacle_info.v_obstacle[i],sel_max_pri_Obstacle.class_id,scan_msg,line_list,text_view,laser_info);
                    line_list_Array.markers.push_back(line_list);
                    line_list_Array.markers.push_back(text_view);
                    vector<std_msgs::ColorRGBA>().swap(line_list.colors);
                    vector<geometry_msgs::Point>().swap(line_list.points);
                }
                showArrowGoal(arrow_goal,sel_max_pri_Obstacle,scan_msg, laser_info);
                line_list_Array.markers.push_back(arrow_goal);
                showArrowAead(arrow_head,scan_msg,obstacle_info.lock_angle,0);
                line_list_Array.markers.push_back(arrow_head);
                showArrowAead(arrow_head,scan_msg,obstacle_info.lock_angle,1);
                line_list_Array.markers.push_back(arrow_head);
                showArrowAead(arrow_head,scan_msg,obstacle_info.lock_angle,2);
                line_list_Array.markers.push_back(arrow_head);
                if (mag_flag)
                {
                    mag_flag=0;
                }
                scan_cluter_markers_pub.publish(line_list_Array);
                vector<visualization_msgs::Marker>().swap(line_list_Array.markers);
            }
        }
        
        boost::this_thread::sleep(boost::posix_time::milliseconds(2)); 
    }
}



void NncaClusterNode:: laserCallback(const sensor_msgs::LaserScan& msg)
{
    
    laser_info.laser_begin_id=0;
    laser_info.C_ranges_max=msg.range_max;
    laser_info.range_min=msg.range_min;
    laser_info.C_ranges_max_set=int(msg.range_max+1);
    laser_info.LASRE_BEAM=msg.ranges.size();
    laser_info.angle_increment=msg.angle_increment;
    if(scan_deque.size()>50)
        scan_deque.pop_front();
    scan_deque.push_back(msg);
};


void NncaClusterNode::orbbecPrioritySet(Obstacle_info & obstacle_info_orbbec)   //深度相机的模拟激光数据用
{

    vector<Obstacle>& v_obstacle_orbbec_tmp=obstacle_info_orbbec.v_obstacle;
    double pri_A=0;
    double c_linear=0;
    double c_angular=0;
    int n_lasers=0;
    for (int i=0; i< v_obstacle_orbbec_tmp.size(); i++)
    {
        //设置线速度 和 角速度
        if (v_obstacle_orbbec_tmp[i].angular<= 5 && v_obstacle_orbbec_tmp[i].angular>= -5 ) 
            v_obstacle_orbbec_tmp[i].angular_speed=0;
        else  if (v_obstacle_orbbec_tmp[i].angular<= 180 && v_obstacle_orbbec_tmp[i].angular> 5 ) 
            v_obstacle_orbbec_tmp[i].angular_speed=0.2;
        else if (v_obstacle_orbbec_tmp[i].angular>= -180 && v_obstacle_orbbec_tmp[i].angular<-5 )
            v_obstacle_orbbec_tmp[i].angular_speed=-0.2;       
        v_obstacle_orbbec_tmp[i].linear_speed=0.2;
        // 设置优先级
        pri_A = v_obstacle_orbbec_tmp[i].pri_A;
        n_lasers=v_obstacle_orbbec_tmp[i].n_lasers;
        if(laser_info_orbbec.C_ranges_max_set==v_obstacle_orbbec_tmp[i].dmax  && pri_A>0)
        {   
            pri_Set_MAX_RANGE(v_obstacle_orbbec_tmp[i],threshold_arr);
        }
        else
        {
            pri_Set(v_obstacle_orbbec_tmp[i],v_obstacle_orbbec_tmp[i].dmin,car_info);
        }
    }
};

void NncaClusterNode::  laserPrioritySet(Obstacle_info & obstacle_info,Obstacle_info & obstacle_info_orbbec)  
{

    vector<Obstacle>& v_obstacle_tmp= obstacle_info.v_obstacle;
    vector<Obstacle>& v_obstacle_orbbec_tmp=  obstacle_info_orbbec.v_obstacle;
    double pri_A=0;
    double c_linear=0;
    double c_angular=0;
    int n_lasers=0;
    int sel_ultrasound_id_first=0;
    int sel_ultrasound_id_second=0;
    laser_info.laser_begin_id=0;
    // 深度相机和激光雷达联合决策 方案一fusion_method==0 松耦合投票决策 暂未实现 不可用
    if (orbbec_scan_msg.header.stamp.toSec()==last_orbbec_laser_time.toSec() || is_orbbec_scan==0 && fusion_method==1)
    // 仅激光雷达决策 或 深度相机和激光雷达融合后的数据联合决策紧耦合的方案，fusion_method==1，is_orbbec_scan为0或1.
    {
        int sel_ultrasound_id=0;
        for (int i=0; i< v_obstacle_tmp.size(); i++)
        {
            //初略设置线速度 和 角速度
            if (v_obstacle_tmp[i].angular<= 5 && v_obstacle_tmp[i].angular>= -5 ) 
                v_obstacle_tmp[i].angular_speed=0;
            else  if (v_obstacle_tmp[i].angular<= 180 && v_obstacle_tmp[i].angular> 5 ) 
                v_obstacle_tmp[i].angular_speed=0.2;
            else if (v_obstacle_tmp[i].angular>= -180 && v_obstacle_tmp[i].angular<-5 )
                v_obstacle_tmp[i].angular_speed=-0.2;       
            v_obstacle_tmp[i].linear_speed=0.2;

            // 设置优先级
            pri_A = v_obstacle_tmp[i].pri_A;
            n_lasers=v_obstacle_tmp[i].n_lasers;
            if(laser_info.C_ranges_max_set==v_obstacle_tmp[i].dmax  && pri_A>0)    //优先级计算入口  (无反射数据的优先级)
            {   
                double  min_delt_angula_first=360;
                double  min_delt_angula_second=360;
                int ul_id=0;
                if (ultrasound_flag==1) //有关超声波的计算
                {
                    ROS_INFO(" ultrasound_flag");
                    for (int j=0;j< 36;j++)
                    {
                        double delt_angular2=abs(v_obstacle_tmp[i].angular-angular_ave_ultrasound[j]);
                        if (delt_angular2< min_delt_angula_first)
                        {
                            sel_ultrasound_id_second=sel_ultrasound_id_first;
                            sel_ultrasound_id_first=j;
                            min_delt_angula_second=min_delt_angula_first;
                            min_delt_angula_first= delt_angular2;
                        }
                    }
                }
                ul_id=floor(sel_ultrasound_id_first/6);
                if((ultrasound_flag==1 && ultrasound_msg.ranges[ul_id]> 0.6) \
                         ||(ultrasound_flag==1 && min_delt_angula_first> 15)||ultrasound_flag==0)  // 不使用超声波时
                {
                    pri_Set_MAX_RANGE(v_obstacle_tmp[i],threshold_arr); 
                }
                else if (abs(ultrasound_msg.ranges[floor(sel_ultrasound_id_first/6)]-ultrasound_Distance[sel_ultrasound_id_first])<0.05 && ultrasound_msg.ranges[sel_ultrasound_id]> 0)
                {
                    ROS_INFO("611 ultrasound_flag");              
                    int ul_id=floor(sel_ultrasound_id_first/6);
                    pri_Set(v_obstacle_tmp[i],ultrasound_msg.ranges[ul_id],car_info); // 使用超声波时，按有反射数据
                }
            }
            else                //优先级计算入口 （有反射数据的）
                pri_Set(v_obstacle_tmp[i],distance_Weigh(v_obstacle_tmp[i]),car_info);
        }
        last_laser_time=scan_msg.header.stamp;
    }
    else
    {
        /*
        nnc_fun_orbbec_laser(laser_info, car_info, orbbec_scan_msg,v_obstacle_orbbec_tmp);
        orbbecPrioritySet(obstacle_info);
        for (int i=0; i< v_obstacle_tmp.size(); i++)
        {
            //设置线速度 和 角速度
            if (v_obstacle_tmp[i].angular<= 5 && v_obstacle_tmp[i].angular>= -5 ) 
                v_obstacle_tmp[i].angular_speed=0;
            else  if (v_obstacle_tmp[i].angular<= 180 && v_obstacle_tmp[i].angular> 5 ) 
                v_obstacle_tmp[i].angular_speed=ROT_VEL_01*2;
            else if (v_obstacle_tmp[i].angular>= -180 && v_obstacle_tmp[i].angular<-5 )
                v_obstacle_tmp[i].angular_speed=-ROT_VEL_01*2;       
            v_obstacle_tmp[i].linear_speed=VEL_X_01*2;
            // 设置优先级
            pri_A = v_obstacle_tmp[i].pri_A;
            n_lasers=v_obstacle_tmp[i].n_lasers;
            if(C_ranges_max_set==v_obstacle_tmp[i].dmax  && pri_A>0)
            {   
                double  min_delt_angula_first=360;
                double  min_delt_angula_second=360;
                int ul_id=0;
                if (ultrasound_flag==1)
                {
                    for (int j=0;j< 36;j++)
                    {
                        double delt_angular2=abs(v_obstacle_tmp[i].angular-angular_ave_ultrasound[j]);
                        if (delt_angular2< min_delt_angula_first)
                        {
                            sel_ultrasound_id_second=sel_ultrasound_id_first;
                            sel_ultrasound_id_first=j;
                            min_delt_angula_second=min_delt_angula_first;
                            min_delt_angula_first= delt_angular2;
                        }
                    }
                }
                ul_id=floor(sel_ultrasound_id_first/6);
                if((ultrasound_flag==1 && ultrasound_msg.ranges[ul_id]> 0.6) \
                         ||(ultrasound_flag==1 && min_delt_angula_first> 15)||ultrasound_flag==0)
                {
                    pri_Set_MAX_RANGE(v_obstacle_tmp[i],threshold_arr);
                }
                else if (abs(ultrasound_msg.ranges[ul_id]-ultrasound_Distance[sel_ultrasound_id_first])<0.05 && ultrasound_msg.ranges[ul_id]>0)
                {
                    pri_Set(v_obstacle_orbbec_tmp[i],ultrasound_msg.ranges[ul_id],car_info);
                }
                ultrasound_flag=0;
            }
            else
            {
                pri_Set(v_obstacle_orbbec_tmp[i],v_obstacle_tmp[i].dmin,car_info);
            }
        }
        last_laser_time=scan_msg.header.stamp;
        last_orbbec_laser_time=orbbec_scan_msg.header.stamp;
        is_orbbec_scan=0;*/
    }
};

Obstacle  NncaClusterNode:: orbbecCmdSet(Obstacle_info & obstacle_info_orbbec,geometry_msgs::TwistStamped &twist)  //深度相机的模拟激光数据用
{
  
    normalModeOrbbec( obstacle_info_orbbec);
    int noUse;
    if (signCount( obstacle_info_orbbec))
        return gpsNormalCmd(noUse,obstacle_info_orbbec,twist);
    else
        return obstacle_info_orbbec.v_obstacle[obstacle_info_orbbec.sel_max_pri_id];
};




Obstacle  NncaClusterNode:: cmdSet(Obstacle_info & obstacle_info,Path_map_info &path_map_info,geometry_msgs::TwistStamped &twist)
{ //   n_runState: 0第一次运行  ，1解冻 ， 2冻结   ,3停车（暂未设置状态转换）
    twist=geometry_msgs::TwistStamped();
    vector<Obstacle>& v_obstacle_tmp= obstacle_info.v_obstacle;
    Obstacle obstacle_tmp;
    obstacle_info.max_pri=0;
    fis_control.set_msg(scan_msg);   //  模糊控制设置激光雷达参数
    fis_control.getDistance_d1_d5();           //  根据激光雷达参数  计算出左、左前、前、右前、右5个方向的5个最短距离
    //SlipModel slipModel();
    //slipModel.judgeSlip(fis_control.d1_d5);
    switch(n_runState)
    {
        case 0:
            ROS_INFO("\n first");
            if (is_current_direction!=1 ||  is_global_direction!=1)
                normalMode(1,obstacle_info);    //  正常模式下 对计算后的优先级选择最好的优先级
            else
                gpsMode(1,obstacle_info ,sensor_info,path_map_info);  //  GPS模式下 对计算后的优先级选择最好的优先级

            n_runState=signCount(obstacle_info);   //等带几帧数据
            return obstacle_info.v_obstacle[obstacle_info.sel_max_pri_id];
        case 1:
            ROS_INFO("\n unfreeze !");
            if (is_current_direction!=1 || is_global_direction!=1)
                normalMode(1,obstacle_info);
            else
                gpsMode(1,obstacle_info ,sensor_info,path_map_info);

            return gpsNormalCmd(n_runState,obstacle_info,twist);   //  根据选择的优先级设置线速度和角速度 （包含使用和不使用模糊控制）
        case 2:
            ROS_INFO("\n freeze !");
            if (is_current_direction!=1 || is_global_direction!=1)
                normalMode(2,obstacle_info);
            else
                gpsMode(2,obstacle_info ,sensor_info,path_map_info);

            return gpsNormalCmd(n_runState,obstacle_info,twist);
        case 3:
            ROS_INFO("\n 停车状态 !");
    }
}

Obstacle  NncaClusterNode:: gpsNormalCmd(int & n_runState,Obstacle_info & obstacle_info,geometry_msgs::TwistStamped &twist)
{
    vector<Obstacle>& v_obstacle_tmp= obstacle_info.v_obstacle;
    double angular_abs;
    int sgn;
    int sum;
    bool is_back;
    obstacle_info.list_sign_cun.pop_front();
    sgn = (v_obstacle_tmp[obstacle_info.sel_max_pri_id].angular >=0) ? (1) : (-1);
    sum=accumulate(obstacle_info.list_sign_cun.begin(),obstacle_info.list_sign_cun.end(),0); 
    obstacle_info.list_sign_cun.push_back(sgn);
    if (sum >=0 && v_obstacle_tmp[obstacle_info.sel_max_pri_id].angular>=0 || (sum <0 && v_obstacle_tmp[obstacle_info.sel_max_pri_id].angular_speed<0)\
            || abs(v_obstacle_tmp[obstacle_info.sel_max_pri_id].angular)<= 5)
    {   //  //  is_back：根据激光雷达左前、前、右前、3个方向的3个最短距离计算是否后退
        is_back= (fis_control.d1_d5[2] <=0.2 || fis_control.d1_d5[1] <= 0.15 || fis_control.d1_d5[3] <=0.15);
        ////////////////////
        if( n_runState==1 && obstacle_info.max_pri > 1)
        {
            obstacle_info.last_sel_Obstacle=v_obstacle_tmp[obstacle_info.sel_max_pri_id];
            if (!is_back)                                    //不使用模糊控制设置线速度和角速度
            {
                angular_abs=abs(v_obstacle_tmp[obstacle_info.sel_max_pri_id].angular);                            
                if (angular_abs<= obstacle_info.turning_angle )
                {
                    n_runState=2;
                    twist=twist_cmd_last;
                    ROS_INFO("v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
                }
                else                                          //不使用模糊控制设置线速度和角速度
                {
                    twist.twist.linear.x  = 0;
                    twist.twist.angular.z  = sgn*0.8;
                    ROS_INFO("v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
                }
            }
            else
            {
                n_runState=1;
                twist.twist.linear.x  = -0.08;
                twist.twist.angular.z  = 0;
                ROS_INFO("v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
            }   
        }
        ////////////////////
        if( n_runState==1 && obstacle_info.max_pri <= 1)   //不使用模糊控制设置线速度和角速度
        {
            ROS_INFO("Dead end ! Need someone's help ! " );
            n_runState=1;
            twist.twist.linear.x  = 0;
            twist.twist.angular.z  = 0;
            ROS_INFO("v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
        }

        ////////////////////
        if( n_runState==2 && obstacle_info.max_pri > 1)
        {
            obstacle_info.last_sel_Obstacle=v_obstacle_tmp[obstacle_info.sel_max_pri_id];
            if (!is_back)
            {
             useFisControl(FIS_FILES,obstacle_info, scan_msg,twist); //使用模糊控制设置线速度和角速度 
            }
            else                                                                //不使用模糊控制设置线速度和角速度
            {
                n_runState=2;
                twist.twist.linear.x  = -0.08;
                twist.twist.angular.z  = 0;
                ROS_INFO("v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
            }

        }
       ////////////////////
        if( n_runState==2 && obstacle_info.max_pri <= 1)        //不使用模糊控制设置线速度和角速度
        {
            n_runState=1;
            twist.twist.linear.x  = 0;
            twist.twist.angular.z  = 0;
            ROS_INFO("v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
        }
        return obstacle_info.last_sel_Obstacle;
    } 
    else 
    {
        twist=twist_cmd_last;
        ROS_INFO("v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
        return obstacle_info.last_sel_Obstacle;
    }
        
}

void NncaClusterNode:: useFisControl(std::vector<char *> FIS_FILES,Obstacle_info & obstacle_info,
                                        const sensor_msgs::LaserScan& msg_tmp,geometry_msgs::TwistStamped &twist)
{
    Obstacle obstacle_tmp=obstacle_info.v_obstacle[obstacle_info.sel_max_pri_id];
    fis_control.set_obstacle(obstacle_tmp);  //模糊控制设置最好选择优先级障碍物参数
    if(obstacle_info.real_distance<= 0.1)  //对真正目标的距离判断是否停止，暂未用到
    {
        twist.twist.linear.x=0;
        twist.twist.angular.z=0;
        ROS_INFO("arrive at goal: v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
        return ;
    }
    //模糊的蔽障控制器
    if (fis_control.d1_d5[2] <= obstacle_info.mid_distance || fis_control.d1_d5[1] <= obstacle_info.left_distance || fis_control.d1_d5[3] <= obstacle_info.left_distance)
    {
       fis_control.returnFisexecOutput(FIS_FILES[0], 0);
        twist.twist.linear.x=car_info.vel_factor*1.2*fis_control.v;
        twist.twist.angular.z=car_info.turn_factor*1.2*car_info.turn_factor*fis_control.w;
        ROS_INFO("obstacle_avoidance mode: v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
        return ;
    }
   //模糊的目标导向控制器
    if(fis_control.d1_d5[2] > obstacle_info.mid_distance && fis_control.d1_d5[1] > obstacle_info.left_distance && fis_control.d1_d5[3] > obstacle_info.left_distance)
    {
        fis_control.returnFisexecOutput(FIS_FILES[1], 1);
        twist.twist.linear.x=car_info.vel_factor*fis_control.v;
        twist.twist.angular.z=car_info.turn_factor*fis_control.w;

        ROS_INFO("goal_oriented mode:  v = %4.4f    w = %4.4f", twist.twist.linear.x,twist.twist.angular.z);
        return ;
    }
}
};

using namespace nnca;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nn_cluter_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");
    NncaClusterNode  nnac_node(nh_, nh_private_); 
    return 0;
}

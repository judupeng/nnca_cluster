#ifndef __NNCA_CLUSTER_NODE__
#define __NNCA_CLUSTER_NODE__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include "tablet_socket_msgs/gear_cmd.h"
#include "tablet_socket_msgs/mode_cmd.h"
#include "tablet_socket_msgs/route_cmd.h"
#include "nnca_cluster/utils.h"
#include "nnca_cluster/struct_define.h"
#include "nnca_cluster/visualization_laser.h"
#include "nnca_cluster/fis_control.h"
//#include "nnca_cluster/generate_bezier.h"
//#include "nnca_cluster/motion.h"


using namespace std;
namespace nnca
{
class NncaClusterNode
{
public:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Publisher scan_cluter_markers_pub;
    ros::Publisher scan_debug_marker_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher point_z_filter_pub;
    ros::Publisher depth_projection_markers_pub;
    ros::Publisher mag_markers_pub;
    //ros::Publisher bezier_controlpoint_path_pub;
    //ros::Publisher bezier_curve_path_pub;
    tf::TransformListener listener;
    tf::StampedTransform transform_;
    sensor_msgs::LaserScan   scan_msg;
    sensor_msgs::LaserScan   orbbec_scan_msg;
    sensor_msgs::LaserScan   merger_scan_msg;
    sensor_msgs::LaserScan   ultrasound_msg;
    sensor_msgs::LaserScan   infrared_msg;
    sensor_msgs::LaserScan   inflat_msg_left;
    sensor_msgs::LaserScan   inflat_msg_right;
    sensor_msgs::MagneticField  imu_mag_msg;
    sensor_msgs::NavSatFix   fix_msg;
    geometry_msgs::TwistStamped  twist_cmd;
    geometry_msgs::TwistStamped  twist_cmd_last;
    FisControl fis_control;
    visualization_msgs::Marker line_list,line_list_orbbec,text_view,text_view_orbbec,arrow_goal,arrow_head;
    visualization_msgs::MarkerArray line_list_Array,line_list_Array_orbbec;
    //vector<point>   gps_route;
    static tf::TransformBroadcaster br;
public:

    std::ofstream SAVE_MOVE_PATH_FILE_;
    std::ofstream SAVE_PLAN_PATH_FILE_;
    State STATE;
    Move_State MOVE_STATE;
    int n_runState;

    double last_cmd_angular_speed_orbbec;
    double last_cmd_linear_speed_orbbec;
    double last_cmd_angular_speed;
    double last_cmd_linear_speed;

    bool  fusion_method; //等于0松耦合先做决策后进行决策投票，暂未实现不可用；等于1紧耦合先融合数据后做一次决策。
    bool  is_orbbec_scan;
    bool  is_orbbec_only;
    bool  is_First;
    bool  is_merger_laser;
    bool  is_freeze_Local;
    bool  is_freeze_Global;
    bool  is_orbbec_First;
    bool  is_orbbec_freeze_Local;
    bool  is_current_direction;
    bool  is_global_direction;
    bool  infrared_flag;
    bool  ultrasound_flag;
    bool  mag_flag;
    bool  is_show;
    bool  is_current_gps_pose;
    
    Car_info      car_info;
    Laser_info    laser_info;
    Laser_info    laser_info_orbbec;
    SensorsInfo   sensor_info;
    Obstacle_info obstacle_info; 
    Obstacle_info obstacle_info_orbbec; 
    Inflate_info  inflate_info;
    Path_map_info path_map_info;
    std::vector<double > threshold_arr;
    std::deque<sensor_msgs::LaserScan> scan_deque;
    char *OA_fis_file;
    char *GO_fis_file;
    std::vector< char *> FIS_FILES;
    ros::Time last_orbbec_laser_time;
    ros::Time last_laser_time;
   
    //
 
public:
    NncaClusterNode(const ros::NodeHandle& nh_, 
                           const ros::NodeHandle& nh_private_);
    virtual ~NncaClusterNode();
    void initializeParams();
    Obstacle  orbbecCmdSet(Obstacle_info & obstacle_info,geometry_msgs::TwistStamped &twist);
    Obstacle  cmdSet(Obstacle_info & obstacle_info,Path_map_info &path_map_info,geometry_msgs::TwistStamped& twist);
    void orbbecPrioritySet(Obstacle_info & obstacle_info_orbbec);
    void laserPrioritySet(Obstacle_info & obstacle_info,Obstacle_info & obstacle_info_orbbec);
    void orbbecLaserCallback(const sensor_msgs::LaserScan& input);
    void laserCallback(const sensor_msgs::LaserScan& msg);
    void mergerLaserCallback(const sensor_msgs::LaserScan& msg);
    void depthPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void ultrasoundCallback(const sensor_msgs::LaserScan& msg);
    void ekfOdomCombinedCallback(const geometry_msgs::PoseWithCovarianceStamped &ekf_Pose_msg);
    void infraredCallback(const sensor_msgs::LaserScan& msg);
    void imuMagCallback(const sensor_msgs::MagneticField& mag_msg);
    void gpsRouteCmdCallback(const tablet_socket_msgs::route_cmd& msg);
    void racm1GpsCallback(const sensor_msgs::NavSatFix msg);
    void yawWithNorthCallback(const std_msgs::Float64 msg);
    void gnssPoseCallback(const geometry_msgs::PoseStamped& pose_msg);
    void racm1PoseCallback(const geometry_msgs::PoseStamped& pose_msg);
    void runState();
    void runCmd(geometry_msgs::TwistStamped &_twist_cmd,geometry_msgs::TwistStamped &_twist_cmd_last);
    void useFisControl(std::vector<char *> FIS_FILES,Obstacle_info & obstacle_info,const sensor_msgs::LaserScan& msg_tmp,geometry_msgs::TwistStamped &twist);
    Obstacle  gpsNormalCmd(int & n_runState,Obstacle_info & obstacle_info,geometry_msgs::TwistStamped &twist);
    void processCluster();

};
}; 
#endif
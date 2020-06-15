

#include "decision/memory_route.h"

/*
MemoryDecision::MemoryDecision(){



}

MemoryDecision::~MemoryDecision(){

}
*/

Obstacle MemoryDecision::choose_walk_yaw(geometry_msgs::PoseStamped current_pose,std::vector<Obstacle> able_walk_yaw){

	Obstacle result_yaw;

	
	if(able_walk_yaw.size()>0){
		
		if(able_walk_yaw.size()>1){
			
			ROS_INFO("walk new way.");
			result_yaw = get_choose_yaw(current_pose,able_walk_yaw);

		}else{
			ROS_INFO("walk old way");	
			result_yaw = able_walk_yaw[0];
		}
	
	
	}else{
	
		ROS_INFO("No Way to go.");
	}

	return result_yaw;

}



Obstacle MemoryDecision::get_choose_yaw(geometry_msgs::PoseStamped current_pose,std::vector<Obstacle> able_walk_yaw){
	
	std::vector<double> able_yaw_set;

	for(int i = 0;i<able_walk_yaw.size();i++){

        	able_yaw_set.push_back(able_walk_yaw[i].angular);

        }

     
        double current_yaw = get_pose_from_xyzw(current_pose);
        
	
	std::vector<double> waypoint_yaw_set;
	get_waypoint_current_angle(current_pose,&waypoint_yaw_set);

	std::vector<int> waypoint_include_set;


	for(int i = 0 ;i<able_yaw_set.size();i++){
	
		double turn_yaw = able_yaw_set[i];

		int waypoint_include_cnt = get_include_waypoint_cnt(current_yaw,turn_yaw,waypoint_yaw_set);

		waypoint_include_set.push_back(waypoint_include_cnt);
	
	}

	int MIN_ = 101;
	int choose_yaw_index = -1;

	for(int i =0;i<waypoint_include_set.size();i++){
	
		
		if(MIN_>waypoint_include_set[i]){
			MIN_ = waypoint_include_set[i];

			choose_yaw_index = i;
		
		}
	
	}

	return able_walk_yaw[choose_yaw_index];

}

int MemoryDecision::get_include_waypoint_cnt(double current_angle,double turn_angle,std::vector<double> waypoint_yaw_set){

	int cnt = 0;
	double tmp;
	
	// if(turn_angle>0){
	// 	tmp = turn_angle;
	// }else{
	// 	tmp = -turn_angle;
	// }
	
	for(int i=0;i<waypoint_yaw_set.size();i++){
		if(turn_angle>=0)
			if(waypoint_yaw_set[i] < 2*tmp && waypoint_yaw_set[i] >0) cnt++; //范围内已走过的轨迹点
		else
			if(waypoint_yaw_set[i] > 2*tmp && waypoint_yaw_set[i] <0) cnt++; //范围内已走过的轨迹点
	}	

	return cnt;

}  


void MemoryDecision::get_waypoint_current_angle(geometry_msgs::PoseStamped current_pose,std::vector<double> *waypoint_yaw_set){

	std::deque<geometry_msgs::PoseStamped> memory_path = walked_waypoints;

	double x1,x2,y1,y2;
	x1 = current_pose.pose.position.x;
	y1 = current_pose.pose.position.y;

        double current_yaw = get_pose_from_xyzw(current_pose);
       

	while(!memory_path.empty()){
	
		geometry_msgs::PoseStamped point = memory_path.back();
		memory_path.pop_back();
		
		x2 = point.pose.position.x;
		y2 = point.pose.position.y;

		//两点夹角
		double angle = atan2((x2-x1),(y2-y1))*180/3.1415926;	

		waypoint_yaw_set->push_back(angle - current_yaw);
	}


}

double MemoryDecision::get_pose_from_xyzw(geometry_msgs::PoseStamped current_pose){



        geometry_msgs::Quaternion orientation = current_pose.pose.orientation;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(orientation, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, current_yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, current_yaw);


        return current_yaw;

}


//判断是否来过
bool MemoryDecision::has_walked(geometry_msgs::PoseStamped current_pose){
	
	std::deque<geometry_msgs::PoseStamped> memory_path = walked_waypoints;
	geometry_msgs::PoseStamped route_point;

	if(memory_path.size()>0){
		
		while(!memory_path.empty()){
			route_point = memory_path.back();
			 memory_path.pop_back();

			if(approx_points(current_pose,route_point)) return true;
				
		}
		
	}

	return false;
}


//近似点
bool MemoryDecision::approx_points(geometry_msgs::PoseStamped a,geometry_msgs::PoseStamped b){
	
	if(get_distance(a,b) < SEARCH_R){//R_半径 

		return true;

	}else{

		return false;
}

}

double MemoryDecision::get_distance(geometry_msgs::PoseStamped a,geometry_msgs::PoseStamped b){

        double x1,x2,y1,y2,z1,z2;

        x1 = a.pose.orientation.x;
        x2 = b.pose.orientation.x;

        y1 = a.pose.orientation.y;
        y2 = b.pose.orientation.y;

        z1 = a.pose.orientation.z;
        z2 = b.pose.orientation.z;

        double res = sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2));

        return res;

}



void MemoryDecision::save_walked_waypoint(geometry_msgs::PoseStamped current_status_pose){

	if(walked_waypoints.size()>WAYPOINT_CNT){

			walked_waypoints.pop_front();
	}

	walked_waypoints.push_back(current_status_pose);
}

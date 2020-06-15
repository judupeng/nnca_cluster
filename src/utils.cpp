
#include "nnca_cluster/utils.h"


/*
     
*/


/*

double get_distance(geometry_msgs::PoseStamped a,geometry_msgs::PoseStamped b){
  
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

*/
int get_closest_distance(nav_msgs::Path &route,geometry_msgs::PoseStamped *current_gps_pose){

    int node_index = -1,MIN_ = 1000000000;

    Point3 current_point;
    current_point.x = current_gps_pose->pose.position.x;
    current_point.y = current_gps_pose->pose.position.y;
    current_point.z = 0.0;
    Point3 tmp_point;
    for(int i = 0;i<route.poses.size();i++){
        tmp_point.x = route.poses[i].pose.position.x;
        tmp_point.y = route.poses[i].pose.position.y;
        tmp_point.z = 0.0;

        double distance = getDistance(current_point,tmp_point);
        //std::cout<<distance<<std::endl;
        if(MIN_ >=distance){

                MIN_ = distance;
                node_index = i;
        }

    }
    //std::cout<<"min:"<<MIN_<<std::endl;
    //std::cout<<"min_index:"<<node_index<<std::endl;
    return node_index;
}

void pro_closest_route(nav_msgs::Path &route,geometry_msgs::PoseStamped *current_gps_pose){

   if(route.poses.size()<0) {
       std::cout<<"Please check your route."<<std::endl;
       return;
       }

    nav_msgs::Path &route_ = route;
    
    int FLAG = get_closest_distance(route_,current_gps_pose);

    if(FLAG == -1){
        std::cout<<"Error happened!"<<std::endl;
        return;
    }else{

        if(FLAG>0){
            
                FLAG = FLAG - 1;
            
        route.poses.erase(route.poses.begin(),route.poses.begin()+FLAG);
        }

        //std::cout<<"run here!"<<std::endl;

        return;
    }


}

Line get_line(Point3 a,Point3 b){

        double m = 0;
        Line l;
        m = b.x - a.x;
        if(0==m){

                l.k = 10000.0;

                l.b = a.y - l.k * a.x;

        }else{

                l.k = (b.y - a.y)/(b.x - a.x);
                l.b = a.y - l.k * a.x;
        }

        return l;

}


void get_cross(Line l1,Line l2,Point3* p){


        if (abs(l1.k - l2.k) > 0.5)
        {
                p->x = (l2.b - l1.b) / (l1.k - l2.k);
                p->y = l1.k * p->x + l1.b;

        }

}


Point3 get_closest_point(Point3 p,Point3 a,Point3 b){


        //solve line equation.

        Line l = get_line(a,b);

        Line l_p;

        l_p.k = -1.0 / l.k;
        l_p.b = p.y - l_p.k * p.x;

        //get cross point.
        Point3 c_p;

        get_cross(l,l_p,&c_p);

        if(c_p.x<a.x && c_p.y<a.y && b.x<c_p.x && b.y<c_p.y){
                return c_p;
        }
        else if(c_p.x>a.x && c_p.y>a.y && b.x>c_p.x && b.y>c_p.y){
                return c_p;
        }
        else{

                return a;
        }

}

Point3 find_closest_ponit3(Point3 current_point,std::vector<Point3> route){

       if(route.size()>1){
        
        Point3 a = route[0];
        Point3 b = route[1];

        return get_closest_point(current_point,a,b);}
       
       else{
	
	return route[0];
	
	}
}


bool route_ready(Point3 current_point,std::vector<Point3> route){

	/*

        if(current_point==NULL) return false;

        Point3 current_point;

        Point3 route_closest_point;
        if(cross){

                route_closest_point = find_closest_ponit3(current_point,route);
        }else{

        route_closest_point = route[0];
                }

		*/
        return true;

}



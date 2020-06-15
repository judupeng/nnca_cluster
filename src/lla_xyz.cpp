

//#include<iomanip>
 

#include "nnca_cluster/lla_xyz.h"
#include "gnss/geo_pos_conv.hpp"
#include <math.h>
static geo_pos_conv geo;

Point3 lla_xyz(double lat,double lon,double alt){

	
	Point3 p;

	geo.llh_to_xyz(lat, lon, alt);
	
	p.x = geo.y()/10;
	p.y = geo.x()/10;
	p.z = geo.z();
	//std::cout<<"x:"<<p.x<<"  y:"<<p.y<<"  z:"<<p.z<<std::endl;
	// printf("x=%2.9f ",p.x);
	// printf(" y=%2.9f \n\n\n",p.y);
	return p;

}


void getRouteXYZ(const tablet_socket_msgs::route_cmd& msg,std::vector<Point3> *route_xyz){


	//std::vector<Point> route_xyz;
	
	for(auto& p : msg.point){
		//tablet_socket_msgs::Waypoint p = msg.point.pop();
		Point3 p_xyz = lla_xyz(p.lat,p.lon,0.0);
		route_xyz->push_back(p_xyz);
	}
	
	
	//return route_xyz;

}
double getDistance(Point3 a,Point3 b){
		double res = -1;
		res = (a.x-b.x) * (a.x-b.x) + (a.y-b.y)*(a.y-b.y);
		res = sqrt(res); 	
		return res;

}

void getPointDistance(std::vector<Point3> route_xyz,std::vector<double> *dis){

	Point3 st_p,en_p;
	int cnt = 0;
	st_p = route_xyz[0];
	for(int j = 1;j<route_xyz.size();j++){

		if(j==1){
			en_p = route_xyz[j];
		}else{
			st_p = route_xyz[j-1];
			en_p = route_xyz[j];
		}
		cnt ++;
		double d = getDistance(st_p,en_p);
		dis->push_back(d);

	}
}

void read_lla_txt(tablet_socket_msgs::route_cmd& msg)
{
	ifstream in("/home/hasee2/catkin_ws/src/nnca_cluster/src/gps.txt");
	string line;
	string a = "";
	string b = "";
	tablet_socket_msgs::Waypoint point_tmp;
	while (getline(in, line))
	{
		stringstream ss(line);
		string token;
		ss >> a;
		ss >> b;
		double lat = atof(a.c_str());
		double lon = atof(b.c_str());
		cout<<lat<<" "<<lon<<endl;
		
		point_tmp.lat=lat;
		point_tmp.lon=lon;
		msg.point.push_back(point_tmp);
		//while (ss >> token)
		//{	}
	}
	in.close();
}

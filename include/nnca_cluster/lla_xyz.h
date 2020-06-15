#ifndef _LLA_XYZ_H_
#define _LLA_XYZ_H_


#include "tablet_socket_msgs/route_cmd.h"
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
struct Point3{

double x;
double y;
double z;

};


struct Line{

   double k;
   double b;

};



Point3 lla_xyz(double lat,double lon,double alt);
void read_lla_txt(tablet_socket_msgs::route_cmd& msg);

void getRouteXYZ(const tablet_socket_msgs::route_cmd& msg,std::vector<Point3> *route_xyz);

void getPointDistance(std::vector<Point3> route_xyz,std::vector<double> *dis);

double getDistance(Point3 a,Point3 b);


#endif
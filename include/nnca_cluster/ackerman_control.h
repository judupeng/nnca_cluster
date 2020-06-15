#ifndef __ACKERMAN_CONTROL__
#define __ACKERMAN_CONTROL__
#include "nnca_cluster/struct_define.h"


class AckermanControl{

    public:
    AckermanControl();
    void setMsg(const sensor_msgs::LaserScan& msg_tmp)
    void getDistance();
    double getMinDistance(int bid,int eid);
    double getMinDistance2(int bid,int eid);

    void makeJudje();

public:
    Obstacle obstacle;
    sensor_msgs::LaserScan scan_msg;
    double d0,d45,d90,d135,d180,d_45,d_90,d_135;
}
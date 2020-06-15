#include "nnca_cluster/ackerman_control.h"

AckermanControl::AckermanControl()
{}


 void AckermanControl::setMsg(const sensor_msgs::LaserScan& msg_tmp)
 {
     scan_msg=msg_tmp;
 }

void AckermanControl::getDistance()
{
    double tmp1,tmp2;
    vector<double>().swap(distances); 
    if(scan_msg.angle_min != 0)
    {
        d90= getMinDistance2(67,112); //  90度， 112~157,67~112, 22~67,-23~22,-68~-23,-68~-113,-113~-158, -158~157,
        d45= getMinDistance2(22,67);//  45度
        d0= getMinDistance2(-23,22);//  0度
        d_45= getMinDistance2(-68,-23); //  -45度
        d_90= getMinDistance2(-113,-68); // -90度
        d_135= getMinDistance2(-158,-113); // -135度
        tmp1=getMinDistance2(-180,-158);
        tmp2=getMinDistance2(157,180);
        if(tmp1<= tmp2) // 180度
            d180= tmp;
        else
             d180= tmp2;
         d135= getMinDistance2(112,157); // 135度
    }
    else
    {
        d90= getMinDistance(67,112);  //  90度 
        d45= getMinDistance(22,67);//  45度
        tmp1=getMinDistance(-23+360,-1+360);
        tmp2=getMinDistance(0,22);
        if(tmp1<= tmp2)
            d0= tmp1);//  0度
        else
            d0= tmp2);//  0度
        d_45= getMinDistance(-68+360,-23+360); //  -45度
        d_90= getMinDistance(-113+360,-68+360);// -90度
        d_135= getMinDistance(-158+360,-113+360);// -135度
        d180=getMinDistance(157,202);// 180度
        d135=getMinDistance(112,157);// 135度
    } 
}


double  AckermanControl::getMinDistance2(int bid,int eid)
{
    double min_d=10000;
    bid=round ((bid*1.0*M_PI/180.0-scan_msg.angle_min)/scan_msg.angle_increment);
    eid=round ((eid*1.0*M_PI/180.0-scan_msg.angle_min)/scan_msg.angle_increment);

    for (int i=bid; i<eid; i++)
    {
        if (scan_msg.ranges[i]< min_d)
            min_d=scan_msg.ranges[i];
    }
    return min_d;
}

double  AckermanControl::getMinDistance(int bid,int eid)
{
    float min_d=10000;
    for (int i=bid; i<eid; i++)
    {
        if (scan_msg.ranges[i]< min_d)
            min_d=scan_msg.ranges[i];
    }
    return min_d;
}

void AckermanControl::makeJudje()
{

}
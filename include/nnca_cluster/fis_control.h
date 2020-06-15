#ifndef __FIS_CONTROL__
#define __FIS_CONTROL__
#include "nnca_cluster/struct_define.h"


class FisControl
{

    public:
    Obstacle obstacle;
    sensor_msgs::LaserScan scan_msg;
    std::vector<double> d1_d5;
    double **dataMatrix;
    double **fisMatrix;
    double **outputMatrix;
    double v;
    double w;
    int data_row_n;
    int data_col_n;
    

    public: 
    FisControl();

    double getMinDistance(int bid,int eid);
    double getMinDistance2(int bid,int eid);
    void  getDistance_d1_d5();

    void createDataMatrix_GO();
    void createDataMatrix_OA3INPUT();
    void createDataMatrix_OA();
    void set_obstacle(Obstacle obstacle_tmp);
    void set_msg(const sensor_msgs::LaserScan& msg_tmp);
 
    void returnFisexecOutput(char *fis_file, int state_fis);

};

#endif
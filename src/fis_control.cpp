#include "nnca_cluster/fis_control.h"
extern "C"{
    #include "../src/fis.c"
    }

 FisControl::FisControl()
 {}

void FisControl::set_obstacle(Obstacle obstacle_tmp)
 {
     obstacle=obstacle_tmp;
 }

 void FisControl::set_msg(const sensor_msgs::LaserScan& msg_tmp)
 {
     scan_msg=msg_tmp;
 }

double  FisControl::getMinDistance2(int bid,int eid)
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

double  FisControl::getMinDistance(int bid,int eid)
{
    float min_d=10000;
    for (int i=bid; i<eid; i++)
    {
        if (scan_msg.ranges[i]< min_d)
            min_d=scan_msg.ranges[i];
    }
    return min_d;
}

void   FisControl::getDistance_d1_d5()
{
    double tmp1,tmp2;
    vector<double>().swap(d1_d5); 
    if(scan_msg.angle_min != 0)
    {
        d1_d5.push_back(getMinDistance2(67,113));
        d1_d5.push_back(getMinDistance2(22,67));
        d1_d5.push_back(getMinDistance2(-23,22));
        d1_d5.push_back(getMinDistance2(-68,-23)); 
        d1_d5.push_back(getMinDistance2(-113,-68)); 
    }
    else
    {
        d1_d5.push_back(getMinDistance(67,113));
        d1_d5.push_back(getMinDistance(22,68));
        tmp1=getMinDistance(-23+360,-1+360);
        tmp2=getMinDistance(0,23);
        if(tmp1<= tmp2)
            d1_d5.push_back(tmp1);
        else
            d1_d5.push_back(tmp2);
        d1_d5.push_back(getMinDistance(-68+360,-22+360)); 
        d1_d5.push_back(getMinDistance(-113+360,-67+360));   
    } 
}

void  FisControl::createDataMatrix_GO()
{
    data_row_n=1;
    data_col_n=2;
    dataMatrix = (DOUBLE **)fisCreateMatrix(data_row_n, data_col_n, sizeof(DOUBLE));
    dataMatrix[0][0]=obstacle.dave;
    dataMatrix[0][1]=obstacle.angular;
    printf(" FIS input: %3.2f    %3.2f   \n",dataMatrix[0][0],dataMatrix[0][1]);
}

void  FisControl::createDataMatrix_OA()
{
	data_row_n=1;
    data_col_n=4;
    dataMatrix = (DOUBLE **)fisCreateMatrix(data_row_n, data_col_n, sizeof(DOUBLE));
    dataMatrix[0][0]=d1_d5[1];
    dataMatrix[0][1]=d1_d5[2];
    dataMatrix[0][2]=d1_d5[3];
    dataMatrix[0][3]=obstacle.angular;
    printf(" FIS input: %3.2f    %3.2f   %3.2f   %3.2f \n",dataMatrix[0][0],dataMatrix[0][1],dataMatrix[0][2],dataMatrix[0][3]);


}

void  FisControl::createDataMatrix_OA3INPUT()
{
	data_row_n=1;
    data_col_n=3;
    dataMatrix = (DOUBLE **)fisCreateMatrix(data_row_n, data_col_n, sizeof(DOUBLE));
    dataMatrix[0][0]=d1_d5[1];
    dataMatrix[0][1]=d1_d5[2];
    dataMatrix[0][2]=d1_d5[3];
    printf(" FIS input: %3.2f    %3.2f   %3.2f \n",dataMatrix[0][0],dataMatrix[0][1],dataMatrix[0][2]);
}

void   FisControl::returnFisexecOutput(char *fis_file, int state_fis)
{
    FIS *fis;
    int i, j;
    int debug = 0;
    int fis_row_n,fis_col_n;
    //char *fis_file, *data_file;
    //data_file = "dataMatrixFile.txt";
    //fis_file = "fisMatrixFile.txt";
    switch(state_fis)
    {
        case 0:
            createDataMatrix_OA();
            break;
        case 1:
            createDataMatrix_GO();
            break;
    }

    fisMatrix = returnFismatrix(fis_file, &fis_row_n, &fis_col_n);

    /* build FIS data structure 建立模糊数据结构*/
    fis = (FIS *)fisCalloc(1, sizeof(FIS));
    /*将fisMatrix中的数据导入到fis中*/
    fisBuildFisNode(fis, fisMatrix, fis_col_n, MF_POINT_N);
    /* error checking 错误检测*/
    if (data_col_n < fis->in_n) {
        printf("Given FIS is a %d-input %d-output system.\n",
            fis->in_n, fis->out_n);
        printf("Given data file does not have enough input entries.\n");
        fisFreeMatrix((void **)dataMatrix, data_row_n);
        fisFreeMatrix((void **)fisMatrix, fis_row_n);
        fisFreeFisNode(fis);
        fisError("Exiting ...");
    }
    /* fisDebugging 调试数据输出*/
    if (debug)
        fisPrintData(fis);

    /* create output matrix 创建输出矩阵  ：要推理的个数 x 输出变量数*/
    outputMatrix = (double **)fisCreateMatrix(data_row_n, fis->out_n, sizeof(double));

    /* evaluate FIS on each input vector 获取输入->开始模糊推理->输出矩阵*/
    for (i = 0; i < data_row_n; i++)
        getFisOutput(dataMatrix[i], fis, outputMatrix[i]);

    /* print output vector 得到输出并打印*/
    for (i = 0; i < data_row_n; i++) {
        for (j = 0; j < fis->out_n; j++)
            printf("%.12f ", outputMatrix[i][j]);
        printf("\n");
    }
    v=outputMatrix[0][0];
    w=outputMatrix[0][1];
    /* clean up memory 清理内存*/
    fisFreeFisNode(fis);
    fisFreeMatrix((void **)dataMatrix, data_row_n);
    fisFreeMatrix((void **)fisMatrix, fis_row_n);
    fisFreeMatrix((void **)outputMatrix, data_row_n);
}


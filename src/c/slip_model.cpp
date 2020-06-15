#include "nnca_cluster/slip_model.h"

/*
//SlipModel slipModel();
//slipModel.judgeSlipLaser(fis_control.d1_d5);
或
SlipModel slipModel();
slipModel.judgeSlipOdomImu(imu_msg,odom);
*/

SlipModel::SlipModel(double wheel_interval)
:
m_isSlip(0),
m_sgnCunt(0),
m_imuFlag(0),
m_yawImuPre(0),
m_yawOdomPre(0),
m_deltaThetaOdom(0),
m_deltaThetaImu(0),
m_ThetaThreshold(0.002),
m_delta_dR(0),
m_delta_dL(0),
m_deltaTheta(0),
m_deltaDistanceOdom(0),
m_deltaDistance(0),
m_wheel_interval(0)
{   
    m_wheel_interval= wheel_interval;
    for(int i=0;i<5;i++)
        m_deltDisArr[i]=0;
}


void SlipModel::setDistanceArr(vector<double> distance_arr)
{
    if(m_distanceArr.size()>=m_num)
        m_distanceArr.pop_back();
    m_distanceArr.push_front(distance_arr);   
}

int SlipModel::cal()
{
    int sgnCunt=0;
    for(int i=1;i<4;i++)
    {
        if(m_distanceArr[0][i]!=m_rang_max_set)
        {
            for(int j=0; j <m_num-1 ; j++)
                m_deltDisArr[i]+=abs(m_distanceArr[j][i]-m_distanceArr[j-1][i]);
            m_deltDisArr[i]= m_deltDisArr[i]/(m_num-1);
            if(m_deltDisArr[i] > m_disturbance )
                sgnCunt++;
        }
        else
        {
            m_deltDisArr[i]=m_rang_max_set;
        }

    }
    printf("%f    %f    %f",m_deltDisArr[1],m_deltDisArr[2],m_deltDisArr[3]);
    m_sgnCunt=sgnCunt;
    if(m_sgnCunt >1 )
        return 1;
    else 
        return 0;
}

int SlipModel::judgeSlipLaser(vector<double> distance_arr,int rang_max_set,double disturbance, int num)
{   // 返回：1代表打滑，2代表 右轮前向打滑 ，3代表 左轮后向打滑
    // 4代表 左轮前向打滑， 5代表 右轮后向打滑 ，0 代表未打滑或不确定（认为未打滑）
    vector<double> distanceArr;
    int sgnCunt=0;
    m_rang_max_set=rang_max_set;
    m_disturbance=disturbance;
    m_num=num;
    setDistanceArr(distance_arr);
    if(m_distanceArr.size()>=m_num)
    {
        return cal();
    }
    else
        return 0;
}

/*
wheel_interval 两轮间距

delta_dR 相邻采用周期右轮走过的弧长
delta_dL 相邻采用周期左轮走过的弧长
m_deltaDistanceOdom  相邻采用周期小车整体的走过的弧长(里程计)
m_deltaThetaOdom 相邻采用周期小车整体的转角变化(里程计)

m_deltaThetaImu   相邻采用周期小车整体的转角变化(Imu)

m_deltaDistance    相邻采用周期小车整体的走过的弧长(实际)
m_deltaTheta     相邻采用周期小车整体的转角变化(实际)


m_deltaDistanceOdom= （m_delta_dR + m_delta_dL）/2
m_deltaThetaOdom= (m_delta_dR - m_delta_dL)/wheel_interval   


第一矫正算法的数学模型为:
m_deltaTheta= m_deltaThetaImu;
m_deltaDistance =  m_delta_dL +  m_deltaThetaImu*wheel_interval/2;

第二矫正算法的数学模型为:
m_deltaTheta= m_deltaThetaImu;
m_deltaDistance =  m_delta_dR +  m_deltaThetaImu*wheel_interval/2;

*/

void SlipModel::calDeltXY(double &deltX,double &deltY )
{
    
    if(m_deltaTheta !=0 )
    {
        deltX= m_deltaDistance* cos(m_yawOdomPre);
        deltY= m_deltaDistance* sin(m_yawOdomPre);
    }
    else
    {
        deltX= 2*m_deltaDistance/m_deltaTheta*sin(0.5*m_deltaTheta)*cos(0.5*m_deltaTheta + m_yawOdomPre);
        deltY= 2*m_deltaDistance/m_deltaTheta*sin(0.5*m_deltaTheta)*sin(0.5*m_deltaTheta + m_yawOdomPre);
    }
    m_yawOdomPre += m_deltaTheta;
}

void SlipModel::correctModel_1(nav_msgs::Odometry &odom)
{
    double deltX,deltY;
    m_deltaTheta= m_deltaThetaImu;
    m_deltaDistance =  m_delta_dL +  m_deltaThetaImu*m_wheel_interval/2;
    calDeltXY(deltX, deltY );
    odom.pose.pose.position.x=deltX+ m_odomPre.pose.pose.position.x;
    odom.pose.pose.position.y=deltY+ m_odomPre.pose.pose.position.y;
    m_odomPre=odom;
}

void SlipModel::correctModel_2(nav_msgs::Odometry &odom)
{
    double deltX,deltY;
    m_deltaTheta= m_deltaThetaImu;
    m_deltaDistance =  m_delta_dR +  m_deltaThetaImu*m_wheel_interval/2;
    calDeltXY(deltX, deltY );
    odom.pose.pose.position.x=deltX+ m_odomPre.pose.pose.position.x;
    odom.pose.pose.position.y=deltY+ m_odomPre.pose.pose.position.y;
    m_odomPre=odom;
}

int SlipModel::calOdomImu(nav_msgs::Odometry &odom)  // 返回：1代表打滑，2代表 右轮前向打滑 ，3代表 左轮后向打滑 
{                            // 4代表 左轮前向打滑， 5代表 右轮后向打滑 ，0 代表未打滑或不确定（认为未打滑）
//////////////////  
    if(abs(m_deltaTheta) <=  m_ThetaThreshold)
    {
        printf("未打滑");
        return 0;
    }
//////////////////  
    if(m_deltaTheta > m_ThetaThreshold)
    {
        if(m_delta_dR > 0 && m_delta_dL >0)
        {
            printf("右轮前向打滑");
            correctModel_1(odom);
            return 2;
        }

        if(m_delta_dR < 0 && m_delta_dL <0)
        {
            printf("左轮后向打滑");
            correctModel_2(odom);
            return 3;
        }
    }
//////////////////   
    if(m_deltaTheta < -m_ThetaThreshold)
    {
           if(m_delta_dR > 0 && m_delta_dL >0)
        {
            printf("左轮前向打滑");
            correctModel_2(odom);
            return 4;
        }
        if(m_delta_dR < 0 && m_delta_dL <0)
        {
            printf("右轮后向打滑");
            correctModel_1(odom);
            return 5;
        }
    }
}

int SlipModel::judgeSlipOdomImu(sensor_msgs::Imu imu_msg,nav_msgs::Odometry &odom)
{
    tf::Quaternion quatImu,quatOdom;
    double roll, pitch, yawImu,yawOdom;//定义存储r\p\y的容器
    double tmp1,tmp2,tmp3,tmp4;
    tf::quaternionMsgToTF(imu_msg.orientation, quatImu);  
    tf::Matrix3x3(quatImu).getRPY(roll, pitch, yawImu);//进行转换

    tf::quaternionMsgToTF(odom.pose.pose.orientation, quatOdom);  
    tf::Matrix3x3(quatOdom).getRPY(roll, pitch, yawOdom);//进行转换
    if (m_imuFlag)
    {
        m_deltaThetaImu = yawImu - m_yawImuPre;
        m_deltaThetaOdom = yawOdom - m_yawOdomPre;
        m_deltaTheta = m_deltaThetaOdom - m_deltaThetaImu;
        tmp1=pow(odom.pose.pose.position.x-m_odomPre.pose.pose.position.x,2);
        tmp2=pow(odom.pose.pose.position.y-m_odomPre.pose.pose.position.y,2);
        tmp3=sqrt(tmp1 +  tmp2);
        tmp4=1.4142;  //tmp4= sqrt(2); 
        if (m_deltaThetaOdom ==0)
        {
            m_deltaDistanceOdom = tmp3;
        }
        else
        {
            m_deltaDistanceOdom = tmp3/tmp4 * m_deltaThetaOdom /sin(0.5*m_deltaThetaOdom);
        }

        m_delta_dR= m_deltaDistanceOdom + m_wheel_interval*m_deltaThetaOdom/2;
        m_delta_dL= m_deltaDistanceOdom - m_wheel_interval*m_deltaThetaOdom/2;
         
        return calOdomImu(odom);
        
    }
    else
    {
        m_odomPre=odom;
        m_imuFlag=1;
        m_yawImuPre=yawImu;
        m_yawOdomPre=yawOdom;
        return 0;
    }

}


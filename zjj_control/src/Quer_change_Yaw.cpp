//这个节点仅仅是用来得到航偏角，还需要一个节点来进行PID输出
#include <ros/ros.h>
#include <tf/tf.h>//用于数据的转换，将四元数转换为航偏角Yaw       
#include <sensor_msgs/Imu.h> //用于获取Imu的数据
#include <std_msgs/Float64.h>
#include "Forword_looking.h"

std_msgs::Float64 k;
void imuCallback(const sensor_msgs::Imu& imu);

int main(int argc,char **argv)
{
    ros::init(argc,argv,"tf_Imu");
    ros::NodeHandle n;
    Forword_looking::Forword sd;

    ros::Subscriber imu=n.subscribe("/imu_data",1,imuCallback);//创建一个订阅者，订阅传感器数据话题,注意订阅者在创建的时候不需要管数据类型，在回调函数里面考虑
    ros::Publisher yaw=n.advertise<std_msgs::Float64>("yaw",1);//创建一个发布者

    while(ros::ok())
    {
         yaw.publish(k);//运行到这一句才开始发布，发布的速率近似为10Hz,k为Yaw
         ros::spin();
    }
    return 0;
}

void imuCallback(const sensor_msgs::Imu& imu)
{
    //sysstd_msgs::Float64 k;

    k.data = tf::getYaw(imu.orientation) * 180.0 / 3.14;

    ROS_INFO("Yaw:%lf",k.data);

    // ros::NodeHandle n;
    // ros::Publisher yaw=n.advertise<std_msgs::Float64>("yaw",1);//创建一个发布者

    // yaw.publish(k);//运行到这一句才开始发布，发布的速率近似为10Hz,k为Yaw
}

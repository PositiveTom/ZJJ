#ifndef FORWORD_LOOKING_H_
#define FORWORD_LOOKING_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "stdio.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <vector>
#include <geometry_msgs/Twist.h>

namespace Forword_looking
{
    class Forword
    {
        public:
            Forword();//构造函数，用于初始化接收路径点和小车自身实际位姿的话题，
            void initForword();
            visualization_msgs::Marker look_for_Forword_Points(const geometry_msgs::Pose& carPose,const nav_msgs::Path map_path);//寻找前方距离Forword_dist的所有点数并得到平均返回的转向角
            double getEta(const geometry_msgs::Pose& carPose);
            double getYawFromPose(const geometry_msgs::Pose& carPose);
            double getCar2GoalDist();
            double getL1Distance(const double& _Vcmd);
            bool foundForwardPt, goal_received, goal_reached;//注意这一行需要自己加进去，因为为自己写的，加在public里面是因为需要其被其他类的function调用来改变
            geometry_msgs::Point odom_goal_pos;//注意这里需要加进去，因为这个类需要官方类的东西

            typedef struct
            {
                double Angle_Target;//要达到的目标位置
                double current_Angle;//现在的位置

                double Kp;
                double Ti;
                double Td;
                
                double now_error;
                double last_error;
                double pre_error;

                double Angle_output;//增量式PID的输出，为角度的输出
                uint16_t turn_output;//将角度的输出转化为PWM的输出

                double T;//采样周期

                int i;//设置初始航偏角为小车要保持的位置
            }kPID;
            kPID sPID;
            uint16_t PID_control(const double Angle_fact);
            void PID_Param_Init();

        private:
            ros::NodeHandle n_;
            ros::Subscriber odom_sub, path_sub, goal_sub;
            ros::Publisher pub_, marker_pub;
            ros::Timer timer1, timer2;
            tf::TransformListener tf_listener;

            nav_msgs::Odometry odom;
            nav_msgs::Path map_path, odom_path;
            geometry_msgs::Twist cmd_vel;
            visualization_msgs::Marker points, line_strip, goal_circle;

            double Forword_dist;//用于改变前瞻性距离
            double Twopoints_dist;//两个路径规划点之间的距离
            double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
            double Gas_gain, baseAngle, Angle_gain, goalRadius;
            int controller_freq, baseSpeed;
            int car_stop;
            int start_loop_flag ;//开始循环的标志
            int start_speed;//开始的速度
            int even_pace;
            bool force_stop;

            void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
            void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
            bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
            bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
            void controlLoopCB(const ros::TimerEvent&);
            void goalReachingCB(const ros::TimerEvent&);
            double getGasInput(const float& current_v);
    };
}



#endif

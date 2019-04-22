#include "Forword_looking.h"
//这里是类方法的定义




Forword_looking::Forword::Forword()
{
    ros::NodeHandle pn("~Forword_looking");//私有空间的节点句柄

    if (pn.hasParam("Forword_dist"))
        pn.getParam("Forword_dist",Forword_dist);
    else
        pn.param("Forword_dist",Forword_dist,0.4);//默认0.4米

    if (pn.hasParam("Twopoints_dist"))
        pn.getParam("Twopoints_dist",Twopoints_dist);
    else
        pn.param("Twopoints_dist",Twopoints_dist,0.4);//两个路径规划点之间默认0.03米
    if (pn.hasParam("lfw"))
        pn.getParam("lfw",lfw);
    else
        pn.param("lfw",lfw,0.13);
    if (pn.hasParam("even_pace"))
    {
        pn.getParam("even_pace",even_pace);
        ROS_INFO("successful");
    }
    else
    {
        pn.param("even_pace",even_pace,1560);
        ROS_INFO("failed");
    }

    pn.param("L", L, 0.26);
    pn.param("Lrv", Lrv, 10.0);
    pn.param("Vcmd", Vcmd, 1.0);
    pn.param("lfw", lfw, 0.13);
    pn.param("lrv", lrv, 10.0);

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("AngleGain", Angle_gain, -1.0);//增加的角度
    pn.param("GasGain", Gas_gain, 1.0);
    pn.param("baseSpeed", baseSpeed, 1575);//基础速度
    pn.param("baseAngle", baseAngle, 90.0);//基础角度
    pn.param("start_loop_flag", start_loop_flag, 0);
    pn.param("start_speed", start_speed, 1560);

    force_stop = true;
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;

    //下面者两个话题声明到时候可以不用，注释掉就行
    odom_sub = n_.subscribe("/odometry/filtered", 1, &Forword_looking::Forword::odomCB, this);
    path_sub = n_.subscribe("/move_base_node/NavfnROS/plan", 1, &Forword_looking::Forword::pathCB,this);
    odom_sub = n_.subscribe("/odometry/filtered", 1, &Forword_looking::Forword::odomCB, this);

    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);

    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &Forword_looking::Forword::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &Forword_looking::Forword::goalReachingCB, this); // Duration(0.05) -> 20Hz

    initForword();//若还有内容待加进去
    car_stop = 0;
}


double Forword_looking::Forword::getGasInput(const float& current_v)
{
    double u = (Vcmd - current_v)*Gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}




double Forword_looking::Forword::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;//car_pose代表小车自身的位置
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}



void Forword_looking::Forword::controlLoopCB(const ros::TimerEvent&)
{
    int count = 100;
    geometry_msgs::Pose carPose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;
    cmd_vel.linear.x = 1500;
    cmd_vel.angular.z = baseAngle;

    if(goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);
        ROS_INFO("Path Point:%ld",map_path.poses.size());  

        if(foundForwardPt)
        {

            //cmd_vel.angular.z = PID_control(eta);
            cmd_vel.angular.z = 0;
            //ROS_INFO("PID:%d  eta:%d",uint16_t(PID_control(eta)),uint16_t(eta);
            /*Estimate Gas Input*/

            if(!goal_reached)
            {
                if(start_loop_flag++ <= 10)
                {

                    double u = getGasInput(carVel.linear.x);
                    
                    cmd_vel.linear.x = start_speed ;



                     start_speed += 4;
                     if(cmd_vel.linear.x > baseSpeed)   cmd_vel.linear.x = baseSpeed;
                     ROS_INFO("baseSpeed = %.2f\tSteering angle = %.2f\t angle_tr=%lf\n",cmd_vel.linear.x,cmd_vel.angular.z,eta*180/M_PI);
                }
                else
                {
                    //ROS_INFO("!goal_reached");
                    double u = getGasInput(carVel.linear.x);                   
                    cmd_vel.linear.x = baseSpeed ;
                    
                    ROS_INFO("Gas = %.2f\tSteering angle = %.2f angle_tr=%lf\n",cmd_vel.linear.x,cmd_vel.angular.z,eta*180/M_PI);
                }  
            }

        }
    }
    if(car_stop > 0)
    {
        start_loop_flag = 0;
        if(carVel.linear.x > 0)
        {

            cmd_vel.linear.x = 1300; //反向刹车
            pub_.publish(cmd_vel);
           // for(int i=0;i<20;i++)
           // {
           //     pub_.publish(cmd_vel);
           //     sleep(0.1);
           //     ROS_INFO("cat stop cmd_vel= %f",cmd_vel.linear.x);
           // }
            
        }
        else
        {
            car_stop = 0;
            cmd_vel.linear.x = 1500;
            pub_.publish(cmd_vel);

            //ROS_INFO("cmd_vel= %f",cmd_vel.linear.x);
        }
    }
    else
    {
        if(!force_stop)
        {
            cmd_vel.linear.x = even_pace;
        }
        pub_.publish(cmd_vel);
        car_stop = 0;
        //ROS_INFO("car run cmd_vel= %f",cmd_vel.linear.x);
    }
}

void Forword_looking::Forword::goalReachingCB(const ros::TimerEvent&)
{
    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();//不明白这个函数是什么意思
        if(car2goal_dist < goalRadius)
        {
            goal_reached = true;
            goal_received = false;
            force_stop = false;
            //ROS_INFO("Goal Reached !");
            car_stop = 100;
        }
    }
}


void Forword_looking::Forword::initForword()
{
    Lfw = goalRadius = getL1Distance(Vcmd);//这个函数不明白是什么意思
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    cmd_vel.linear.x = 1500; // 1500 for stop
    cmd_vel.angular.z = baseAngle;

    ROS_INFO("[param] baseSpeed: %d", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);




        points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green,点是绿色的
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue 线条是蓝色的
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow 目标圆是黄色的
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;

    PID_Param_Init();
}


double Forword_looking::Forword::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.34)
        L1 = 3 / 3.0;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}


//接收里程计的话题消息,这个同上面配套注释
void Forword_looking::Forword::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}

//接收路径点的话题消息，这个同上面配套注释
void Forword_looking::Forword::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
}

//这一行计算与世界坐标系的航偏角的，到时候配套注释掉，因为里面有现成的
double Forword_looking::Forword::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

//这一行计算与世界坐标系的航偏角的，到时候配套注释掉，因为里面有现成的
bool Forword_looking::Forword::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)//wayPt为路径点,carPose里程计得到的小车位姿
{
    /*小车->世界坐标系*/
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose); //这里的旋转角是相对于哪个坐标系的角，暂定世界坐标系
   
    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;//要弄懂这里为什么这样就是向前
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}

//这一行计算与世界坐标系的航偏角的，到时候配套注释掉，因为里面有现成的
bool Forword_looking::Forword::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);//dist为小车目前点与路径点的位置

    if(dist < lfw)
        return false;
    else if(dist >= lfw)//为什么前方目标点距小车的距离大于一个值lfw就要返回true，然后就算跳出了遍历,这里只有当前方的点距离小车的距离大于一定的值才能被判定为前方的点，双重判断
        return true;
}

//在这里借助visualization_msgs::Marker，因为需要用到里面的存储点集的数组
visualization_msgs::Marker Forword_looking::Forword::look_for_Forword_Points(const geometry_msgs::Pose& carPose,const nav_msgs::Path map_path)
{
    visualization_msgs::Marker Points;//用于存储前方取的数组点集

    int Forword_Points_number = ((int)(Forword_dist/Twopoints_dist*10)%10 >= 5) ? ((int)Forword_dist/Twopoints_dist+1) : ((int)Forword_dist/Twopoints_dist);//四舍五入得到的是往前应该数的点数

    geometry_msgs::Point carPose_pos = carPose.position;//小车的位置都是相对于世界坐标系而言的，所以得到的航偏角也是相对于世界坐标系而言的
    // double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached)
    {
        for(int i=0; i<map_path.poses.size(); ++i)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];//从起始位置开始遍历
            geometry_msgs::PoseStamped odom_path_pose;
            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;//这个点为选出来的路径点
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);//判断是否为前方的点

                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)//进入此if语句即标志着找到了前进的点了
                    {
                        if(i+Forword_Points_number <= map_path.poses.size()-1)
                        {
                            // Points.points[0].x=0;//这种表示非常重要，要熟记
                            for(int j=0; j<Forword_Points_number; j++)
                            {
                                Points.points[j] = map_path.poses[i+j].pose.position; //这个数组存储的是综合提取出来的路径点集
                            }
                        }
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        break;//break跳出for循环
                    }
                }
            
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
    }


    points.points.clear();
    line_strip.points.clear();
    
    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);//这里发布的是小车现在的位姿
    marker_pub.publish(line_strip);


    //在此以上，已经得到了点的集合
    return Points;
}


//这里还需要改动一些
double Forword_looking::Forword::getEta(const geometry_msgs::Pose& carPose)//carPose为小车的位姿信息
{   
    int Forword_Points_number=((int)(Forword_dist/Twopoints_dist*10)%10 >= 5) ? ((int)Forword_dist/Twopoints_dist+1) : ((int)Forword_dist/Twopoints_dist);
    //创建一个存储所有点偏角的数组
    std::vector<double> points_eta(Forword_Points_number);
    double carPose_yaw = getYawFromPose(carPose);//得到航偏角
    double all_eta = 0;
    geometry_msgs::Point odom_car2WayPtVec;
    
    visualization_msgs::Marker forwardPt = look_for_Forword_Points(carPose,map_path);//需要找多一些点，然后返回回来求转角的均值

    for(int i=0; i<Forword_Points_number; i++)
    {
        odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.points[i].x - carPose.position.x) + sin(carPose_yaw)*(forwardPt.points[i].y - carPose.position.y);
        odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.points[i].x - carPose.position.x) + cos(carPose_yaw)*(forwardPt.points[i].y - carPose.position.y);
        
        points_eta[i] = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
        all_eta += points_eta[i];
    }

    
    // geometry_msgs::Point odom_car2WayPtVe = odom_car2WayPtVec.points[1];
    // odom_car2WayPtVe.y = 1;
    // odom_car2WayPtVe.x = 1;
    // double eta = atan2(odom_car2WayPtVe.y,odom_car2WayPtVe.x);
    return all_eta/Forword_Points_number;//至此已经得到了舵机需要转动的角度了
}







// //using namespace Forword_looking;
// static int even_pace = 1500;



// static kPID sPID;


void Forword_looking::Forword::PID_Param_Init()
{
    sPID.Angle_Target = 0;//要从IMU读取初始值

    sPID.now_error = 0;
    sPID.last_error = 0;
    sPID.pre_error = 0;

    sPID.Kp = 3.5;
    sPID.Ti = 0;
    sPID.Td = 0;

    sPID.T = 0.002;//根据实际设置采样频率,2ms发布一次

    sPID.i = 0;
}


// 采用增量式PID,只需要采用3次误差的值
uint16_t Forword_looking::Forword::PID_control(const double Angle_fact)
{
    sPID.now_error = Angle_fact*180/3.1415926;

    if(sPID.now_error < -45)
    {
        sPID.now_error = -45;
    }
    else if(sPID.now_error > 45)
    {
        sPID.now_error = 45;
    }



    if(sPID.now_error < 0)//意味着右拐
    {
        sPID.Angle_output = sPID.Kp*sPID.now_error + sPID.Kp*sPID.Td*(sPID.now_error - sPID.last_error)/sPID.T;

        if (sPID.Angle_output < -45)
        {
            sPID.Angle_output = -45;
        }
        else if(sPID.Angle_output > 0)
        {
            sPID.Angle_output = 0;
        }

    }
    else if(sPID.now_error > 0)
    {
        sPID.Angle_output = sPID.Kp*sPID.now_error + sPID.Kp*sPID.Td*(sPID.now_error - sPID.last_error)/sPID.T;

        if(sPID.Angle_output > 45)
        {
            sPID.Angle_output = 45;
        }
        else if(sPID.Angle_output <0 )
        {
            sPID.Angle_output = 0;
        }
    }

    //进行前后误差的工作交接，保存3次误差
    sPID.pre_error = sPID.last_error;
    sPID.last_error = sPID.Angle_Target;

    sPID.turn_output = (int)(10.0/9.0*sPID.Angle_output + 90);

    return sPID.turn_output;//返回的值为40到140
}
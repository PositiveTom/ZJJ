//hello

#include "ros/ros.h"
#include "nav_msgs/Path.h" 
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>


class caculate_Points
{
    public:
        caculate_Points();//析构函数，初始化声明一些订阅者，发布者
        void receive_path_point(const nav_msgs::Path::ConstPtr& pathMsg);//接收路径点
        void receive_odom(const nav_msgs::Odometry::ConstPtr& odomMsg);//接收小车坐标
        double getEtas(const geometry_msgs::Pose& carPose);//得到应该的转角
        int find_forword_point(const geometry_msgs::Pose& carPose);//找到小车前面的一个路径点在数组里面的下标值
        double caculate_average_yaw(unsigned int first_point_index,unsigned int step,unsigned int num,const geometry_msgs::Pose& carPose);//first_point_index代表第一个路径点的数组下标值，step为隔多远取下一个点，num为取多少个点
        double getYawFromPose(const geometry_msgs::Pose& carPose);//得到小车相对于odom坐标系的航偏角
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);//判断是不是前面的路径点
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);//判断是否到达那个点
        double getL1Distance(const double& _Vcmd);
        void caculate_average_yaw(const ros::TimerEvent&);    

    private:
        ros::NodeHandle n_;
        ros::Subscriber sub_path_point,sub_car_pos;
        ros::Timer timer1;

        tf::TransformListener listener;

        nav_msgs::Path local_path;//用于接收路径点
        nav_msgs::Odometry car_pose;//用于接收小车位姿

        unsigned int step;//取点的步长
        unsigned int num;//取多少个点

        double Lfw,Vcmd;

};

caculate_Points::caculate_Points()//相关参数的初始化
{
    ros::NodeHandle p("~");

    sub_path_point = p.subscribe("/move_base/TrajectoryPlannerROS/local_plan", 1, &caculate_Points::receive_path_point,this);
    sub_car_pos = p.subscribe("/odom", 1, &caculate_Points::receive_odom,this);
    timer1 = p.createTimer(ros::Duration((1.0)/20), &caculate_Points::caculate_average_yaw, this); // Duration(0.05) -> 20Hz

    p.param("Vcmd", Vcmd, 1.0);
    step = 1;
    num = 10;

    Lfw = getL1Distance(Vcmd);

    ROS_INFO("hello");
}

void caculate_Points::caculate_average_yaw(const ros::TimerEvent&)
{
    geometry_msgs::Pose carPose = car_pose.pose.pose;
    double eta = getEtas(carPose);
    ROS_INFO("%lf",eta);
}



double caculate_Points::getL1Distance(const double& _Vcmd)
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


// nav_msgs::Path local_path;//创建一个变量用来接收路径数据
// nav_msgs::Odometry car_pose;
// geometry_msgs::Pose carPose;
// geometry_msgs::PoseStamped odom_path_pose;

// geometry_msgs::PointStamped;
void caculate_Points::receive_path_point(const nav_msgs::Path::ConstPtr& pathMsg)//获取map坐标系下的路径点
{
    local_path = *pathMsg;
    ROS_INFO("I receive path point:%ld",local_path.poses.size());
}

void caculate_Points::receive_odom(const nav_msgs::Odometry::ConstPtr& odomMsg)//获取小车的odom下的坐标
{
    car_pose = *odomMsg;
    ROS_INFO("I receive car_pose");
}

//思路：首先找到前面的第一个点，然后多取前方几个点计算yaw值并给权重，调权重就可以了
double caculate_Points::getEtas(const geometry_msgs::Pose& carPose)
{
    int first_point_index = find_forword_point(carPose);
    if(first_point_index != -1)
    {
        double average_yaw = caculate_average_yaw(first_point_index,step,num,carPose);
        return average_yaw;
    }
    else 
    {
        return -1;
    }
}

int caculate_Points::find_forword_point(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;//得到小车的位置
    double car_theta = getYawFromPose(carPose);

    geometry_msgs::Point first_forwardPt;//小车前面的第一个路径点
    // geometry_msgs::Point odom_car2WayPtVec;//odom坐标系下的路径点
    int first_index;//记录寻找的第一个目标点的下标

    for(int i =0; i< local_path.poses.size(); i++)
    {
        geometry_msgs::PoseStamped local_path_pose = local_path.poses[i];//取出路径点数组里面的一个路径点,数据类型决定了要这样取点
        geometry_msgs::PoseStamped odom_path_pose;//需要将map坐标系下的路径点转化为odom坐标系下的路径点

        try
        {
            // tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);//坐标系转换，由于现在运行出错，实际运行再试验
            // geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
            geometry_msgs::Point odom_path_wayPt = local_path_pose.pose.position;
            bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);
            if(_isForwardWayPt)
            {
                bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                if(_isWayPtAwayFromLfwDist)
                {
                    first_index = i;
                    return first_index;
                }
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        
    }

    return -1;
}

double caculate_Points::caculate_average_yaw(unsigned int first_point_index,unsigned int step,unsigned int num,const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;//小车odom下的坐标

    double carPose_yaw = getYawFromPose(carPose);
    double yaw_all = 0;//转角总值
    double yaw_one_time = 0;//每一次计算得到的转角值

    geometry_msgs::Point odom_car2WayPtVec;//odom坐标系下的路径点

    for(int i=first_point_index; i<(first_point_index+step*num); i = i+step)
    {
        geometry_msgs::PoseStamped local_path_pose = local_path.poses[i];
        geometry_msgs::PoseStamped odom_path_pose;

        try
        {
            // tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);//map_path_pose为输入，odom_path_pose为输出，误差矫正坐标转换
            // geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
            geometry_msgs::Point odom_path_wayPt = local_path_pose.pose.position;

            odom_car2WayPtVec.x = cos(carPose_yaw)*(odom_path_wayPt.x - carPose_pos.x) + sin(carPose_yaw)*(odom_path_wayPt.y - carPose_pos.y);
            odom_car2WayPtVec.y = -sin(carPose_yaw)*(odom_path_wayPt.x - carPose_pos.x) + cos(carPose_yaw)*(odom_path_wayPt.y - carPose_pos.y);
            yaw_one_time = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
            yaw_all += yaw_one_time;

        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    return yaw_all/num;
}

double caculate_Points::getYawFromPose(const geometry_msgs::Pose& carPose)
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
 
/*下面为判断是不是前面的路径点，为一个关键程序,传参需要同一个坐标系下的小车坐标和路径点坐标*/
bool caculate_Points::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;//已经证明了
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}

/*下面为判断是否到了这个路径点*/
bool caculate_Points::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);//dist为小车目前点与路径点的位置

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)//为什么前方目标点距小车的距离大于一个值lfw就要返回true，然后就算跳出了遍历
        return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Deal_with_Path_Point");
    ros::NodeHandle n;
    caculate_Points task;
    ros::spin();
    return 0;
}




    // ros::Subscriber sub = n.subscribe("/move_base/TrajectoryPlannerROS/local_plan", 1, receive_path_point);
    // ros::Subscriber sub2 = n.subscribe("/odom", 1, receive_odom);


    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     tf::StampedTransform transform;
    //     try
    //     {
    //         // geometry_msgs::PoseStamped map_path_pose = local_path.poses[1];
    //         // listener.transformPose("odom",ros::Time(0) , map_path_pose, "base_footprint" ,odom_path_pose);
    //         // listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(3.0));
    //         // listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);//这一行可以查出base_footprint坐标系在odom坐标系下的位置
    //         // listener.transformPoint("/base_footprint",local_path.poses[1]);
    //     }
    //     catch (tf::TransformException &ex)
    //     {
    //         ROS_ERROR("%s",ex.what());
    //         ros::Duration(1.0).sleep();
    //     }
    //     ROS_INFO("%lf   %lf",transform.getOrigin().y(),transform.getOrigin().x());
    // }

//
// Created by ubuntu on 24-7-1.
//
#include <ros/ros.h>
#include <rm_msgs/MoveL.h>
#include <rm_msgs/MoveJ_P.h>
#include <rm_msgs/Set_Force_Position.h>
#include <rm_msgs/Plan_State.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

//全局变量，用于阻止MoveL重复执行
int arrival_flag = 0;
bool SetForcePosition = false;
bool StopForcePosition = false;
ros::Publisher moveL_pub;

// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void PlanState_Callback(const rm_msgs::Plan_State::ConstPtr& msg)
{
    // 将接收到的消息打印出来，显示机械臂是否完成运动
    if(msg->state)
    {
        if(arrival_flag == 0)
        {
            arrival_flag = 1;
            ROS_INFO("The first trajectory MoveJ_P has been executed");
        }
        else if(arrival_flag == 1)
        {
            ROS_INFO("MoveL has been executed");
            arrival_flag = 2;
        }
        
    } else {
        ROS_INFO("*******Plan State Fail");
    }

}

void SetForcePosition_Callback(const std_msgs::Bool& msg)
{
    if(msg.data)
    {
        SetForcePosition = true;
        ROS_INFO("ForcePosition set success");
    }
    else
    {
        ROS_ERROR("ForcePosition set faild");
    }
}

void StopForcePosition_Callback(const std_msgs::Bool& msg)
{
    if(msg.data)
    {
        ROS_INFO("StopForcePosition set success");
        StopForcePosition = true;
    }
    else
    {
        ROS_ERROR("StopForcePosition set faild");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "api_moveL_demo");
    ros::NodeHandle nh;
    // 声明spinner对象，参数3表示并发线程数，默认处理全局Callback队列
    ros::AsyncSpinner spin(2);
    // 启动两个spinner线程并发执行可用回调 
    spin.start();
    std_msgs::Empty empty_value;


    /*
     * 1.相关初始化
     */
    // 空间规划指令Publisher
    ros::Publisher MoveJ_P_pub = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);

    // 设置六维力指令Publisher
    ros::Publisher SetForcePosition_pub = nh.advertise<rm_msgs::Set_Force_Position>("/rm_driver/SetForcePosition_Cmd", 10);

    // 设置停止六维力指令Publisher
    ros::Publisher StopForcePosition_pub = nh.advertise<std_msgs::Empty>("/rm_driver/StopForcePosition_Cmd", 10);

    // 直线规划指令Publisher
    moveL_pub = nh.advertise<rm_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);

    // 订阅机械臂执行状态话题
    ros::Subscriber PlanState_sub = nh.subscribe("/rm_driver/Plan_State", 10, PlanState_Callback);
    // 订阅机械臂力位混合设置结果
    ros::Subscriber SetForcePosition_sub = nh.subscribe("/rm_driver/SetForcePosition_Result", 10, SetForcePosition_Callback);
    // 订阅机械臂力位混合停止结果
    ros::Subscriber StopForcePosition_sub = nh.subscribe("/rm_driver/StopForcePosition_Result", 10, StopForcePosition_Callback);

    ros::Duration(2.0).sleep();


    /**
     * 2.通过MoveJ_P指令控制机械臂运动到初始位姿
     */
    ROS_INFO("MoveJ_P start.");
    // 定义一个MoveJ_P指令的目标位姿
    rm_msgs::MoveJ_P moveJ_P_TargetPose;

    moveJ_P_TargetPose.Pose.position.x = -0.355816;
    moveJ_P_TargetPose.Pose.position.y = -0.000013;
    moveJ_P_TargetPose.Pose.position.z = 0.222814;
    moveJ_P_TargetPose.Pose.orientation.x = 0.995179;
    moveJ_P_TargetPose.Pose.orientation.y = -0.094604;
    moveJ_P_TargetPose.Pose.orientation.z = -0.025721;
    moveJ_P_TargetPose.Pose.orientation.w = 0.002349;
    moveJ_P_TargetPose.speed = 0.2;
    moveJ_P_TargetPose.trajectory_connect = 0;
    
    // 发布空间规划指令使机械臂运动到初始位姿
    MoveJ_P_pub.publish(moveJ_P_TargetPose);
    while((arrival_flag == 0)&&(ros::ok()));    //等待moveJP到达位置
    rm_msgs::Set_Force_Position setForce_value;
    setForce_value.sensor = 1;                  //六维力
    setForce_value.mode = 0;                    //工作坐标系
    setForce_value.direction = 2;               //沿Z轴
    setForce_value.N = 5;                       //给5N的力
    ROS_INFO("Set Force Position start.");
    SetForcePosition_pub.publish(setForce_value);
    ros::Duration(1.0).sleep();
    while((SetForcePosition == false)&&(ros::ok())); //等待力位混合设置成功
    // 通过MoveL指令控制机械臂直线运动
    // 定义一个MoveL指令的目标位姿（该位姿通过调整初始位姿Z坐标获得），使机械臂末端沿Y轴运动。
    rm_msgs::MoveL moveL_TargetPose;
    moveL_TargetPose.Pose.position.x = -0.255816;
    moveL_TargetPose.Pose.position.y = -0.000013;
    moveL_TargetPose.Pose.position.z = 0.222814;
    moveL_TargetPose.Pose.orientation.x = 0.995179;
    moveL_TargetPose.Pose.orientation.y = -0.094604;
    moveL_TargetPose.Pose.orientation.z = -0.025721;
    moveL_TargetPose.Pose.orientation.w = 0.002349;
    moveL_TargetPose.speed = 0.2;
    moveL_TargetPose.trajectory_connect = 0;

    ROS_INFO("MoveL start");
    moveL_pub.publish(moveL_TargetPose);
    while((arrival_flag == 1)&&(ros::ok()));         //等待moveL到达位置
    ROS_INFO("Stop Force Position start.");
    StopForcePosition_pub.publish(empty_value);
    while((StopForcePosition == false)&&(ros::ok()));//等待停止力位混合设置成功
    ROS_INFO("*******All command is over please click ctrl+c end*******");
    ros::waitForShutdown();

    return 0;
}



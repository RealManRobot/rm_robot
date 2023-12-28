//
// Created by ubuntu on 22-6-22.
//
#include <ros/ros.h>
#include <rm_msgs/MoveL.h>
#include <rm_msgs/MoveJ_P.h>
#include <rm_msgs/Plan_State.h>


//全局变量，用于阻止MoveL重复执行
int flag = 0;

ros::Publisher moveL_pub;

// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
{
    // 将接收到的消息打印出来，显示机械臂是否完成运动
    if(msg->state)
    {
        if(flag == 0)
        {
            ROS_INFO("The first trajectory has been executed");
            ROS_INFO("Prepare to execute Instruction MoveL");

            //延时2秒，确保机械臂执行完上一条轨迹后稳定
            ros::Duration(2.0).sleep();

            // 通过MoveL指令控制机械臂直线运动
            // 定义一个MoveL指令的目标位姿（该位姿通过调整初始位姿Z坐标获得），使机械臂末端垂直上升
            rm_msgs::MoveL moveL_TargetPose;
            moveL_TargetPose.Pose.position.x = -0.317239;
            moveL_TargetPose.Pose.position.y = 0.120903;
            moveL_TargetPose.Pose.position.z = 0.295765;
            moveL_TargetPose.Pose.orientation.x = -0.983404;
            moveL_TargetPose.Pose.orientation.y = -0.178432;
            moveL_TargetPose.Pose.orientation.z = 0.032271;
            moveL_TargetPose.Pose.orientation.w = 0.006129;
            moveL_TargetPose.speed = 0.6;

            // 发布位姿
            moveL_pub.publish(moveL_TargetPose);

            flag = 1;
        }
        else if(flag == 1)
        {
            ROS_INFO("MoveL has been executed");
            flag = 2;
        }
        
    } else {
        ROS_INFO("*******Plan State Fail");
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "api_moveL_demo");
    ros::NodeHandle nh;
    // 声明spinner对象，参数3表示并发线程数，默认处理全局Callback队列
    ros::AsyncSpinner spin(3);
    // 启动两个spinner线程并发执行可用回调 
    spin.start();


    /*
     * 1.相关初始化
     */
    // 空间规划指令Publisher
    ros::Publisher moveJ_P_pub = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);

    // 直线规划指令Publisher
    moveL_pub = nh.advertise<rm_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);

    // 订阅机械臂执行状态话题
    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);

    ros::Duration(2.0).sleep();


    /**
     * 2.通过MoveJ_P指令控制机械臂运动到初始位姿
     */

    // 定义一个MoveJ_P指令的目标位姿
    rm_msgs::MoveJ_P moveJ_P_TargetPose;
    moveJ_P_TargetPose.Pose.position.x = -0.317239;
    moveJ_P_TargetPose.Pose.position.y = 0.120903;
    moveJ_P_TargetPose.Pose.position.z = 0.255765;
    moveJ_P_TargetPose.Pose.orientation.x = -0.983404;
    moveJ_P_TargetPose.Pose.orientation.y = -0.178432;
    moveJ_P_TargetPose.Pose.orientation.z = 0.032271;
    moveJ_P_TargetPose.Pose.orientation.w = 0.006129;
    moveJ_P_TargetPose.speed = 0.3;
    
    // 发布空间规划指令使机械臂运动到初始位姿
    moveJ_P_pub.publish(moveJ_P_TargetPose);

    ros::waitForShutdown();

    return 0;
}



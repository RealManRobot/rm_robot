//
// Created by ubuntu on 22-6-22.
//
#include <ros/ros.h>
#include <rm_msgs/MoveJ_P.h>
#include <rm_msgs/Plan_State.h>



// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
{
    // 将接收到的消息打印出来，显示机械臂是否完成运动
    if(msg->state)
    {
        ROS_INFO("*******Plan State OK");
    } else {
        ROS_INFO("*******Plan State Fail");
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "api_moveJ_P_demo");
    ros::NodeHandle nh;

    // 声明spinner对象，参数2表示并发线程数，默认处理全局Callback队列
    ros::AsyncSpinner spin(2);
    // 启动两个spinner线程并发执行可用回调 
    spin.start();


    /*
     * 1.相关初始化
     */
    // 空间规划指令Publisher
    ros::Publisher moveJ_P_pub = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);

    // 订阅机械臂执行状态话题
    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);

    ros::Duration(2.0).sleep();


    /**
     * 2.通过MoveJ_P指令控制机械臂运动到初始位置
     */

    // 定义一个MoveJ_P指令的目标位姿
    rm_msgs::MoveJ_P moveJ_P_TargetPose;
    moveJ_P_TargetPose.Pose.position.x = -0.317239;
    moveJ_P_TargetPose.Pose.position.y = 0.120903;
    moveJ_P_TargetPose.Pose.position.z = 0.255765 + 0.04;
    moveJ_P_TargetPose.Pose.orientation.x = -0.983404;
    moveJ_P_TargetPose.Pose.orientation.y = -0.178432;
    moveJ_P_TargetPose.Pose.orientation.z = 0.032271;
    moveJ_P_TargetPose.Pose.orientation.w = 0.006129;
    moveJ_P_TargetPose.speed = 0.2;

    // 发布位姿
    moveJ_P_pub.publish(moveJ_P_TargetPose);

    ros::waitForShutdown();

    return 0;
}



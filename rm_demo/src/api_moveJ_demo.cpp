//
// Created by ubuntu on 22-6-22.
//
#include <ros/ros.h>
#include <rm_msgs/MoveJ.h>
#include <rm_msgs/Plan_State.h>

int arm_dof;
 
// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg->state)
    {
        ROS_INFO("*******Plan State OK");
    } else {
        ROS_INFO("*******Plan State Fail");
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "api_moveJ_demo");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh_("~");
    private_nh_.param<int>        ("Arm_Dof",         arm_dof,           6);
    ROS_INFO("*******arm_dof = %d", arm_dof);
    // 声明spinner对象，参数2表示并发线程数，默认处理全局Callback队列
    ros::AsyncSpinner spin(2);
    // 启动两个spinner线程并发执行可用回调 
    spin.start();


    /*
     * 1.相关初始化
     */
    // 空间规划指令Publisher
    ros::Publisher moveJ_pub = nh.advertise<rm_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);

    // 订阅机械臂执行状态话题
    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);

    ros::Duration(2.0).sleep();
    /**
     * 2.控制机械臂运动到目标位姿
     */
    //定义一个MoveJ指令的目标位姿
    rm_msgs::MoveJ moveJ_BeginPose;
    if(arm_dof == 6)
    {
        moveJ_BeginPose.joint.resize(6);
        moveJ_BeginPose.joint[0] = -0.360829;
        moveJ_BeginPose.joint[1] = 0.528468;
        moveJ_BeginPose.joint[2] = 1.326293;
        moveJ_BeginPose.joint[3] = -0.000454;
        moveJ_BeginPose.joint[4] = 1.221748;
        moveJ_BeginPose.joint[5] = 0.000052;
    }
    else if(arm_dof == 7)
    {
        moveJ_BeginPose.joint.resize(7);
        moveJ_BeginPose.joint[0] = 0.176278;
        moveJ_BeginPose.joint[1] = 0.0;
        moveJ_BeginPose.joint[2] = 0.3543;
        moveJ_BeginPose.joint[3] = 0.53;
        moveJ_BeginPose.joint[4] = 0.00873;
        moveJ_BeginPose.joint[5] = 0.3595;
        moveJ_BeginPose.joint[6] = 0.3595;
    }
    moveJ_BeginPose.speed = 0.3;
    //发布空间规划指令使机械臂运动到目标位姿
    moveJ_pub.publish(moveJ_BeginPose);

    ros::waitForShutdown();

    return 0;
}

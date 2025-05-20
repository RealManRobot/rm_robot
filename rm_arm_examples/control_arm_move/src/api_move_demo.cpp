//
// Created by ubuntu on 24-7-1.
//
#include <ros/ros.h>
#include <string>
#include <rm_msgs/MoveJ.h>
#include <rm_msgs/MoveJ_P.h>
#include <rm_msgs/MoveL.h>
#include <rm_msgs/MoveC.h>
#include <rm_msgs/Plan_State.h>

int arm_dof;
std::string arm_type;
int arrival_flag = 0;

// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg->state)
    {
        if(arrival_flag == 0)
        {
            arrival_flag = 1;
            ROS_INFO("*******Plan MoveJ State OK");
        }
        else if(arrival_flag == 1)
        {
            arrival_flag = 2;
            ROS_INFO("*******Plan MoveJ_P State OK");
        }
        else if(arrival_flag == 2)
        {
            arrival_flag = 3;
            ROS_INFO("*******Plan MoveL State OK");
        }
        else if(arrival_flag == 3)
        {
            arrival_flag = 0;
            ROS_INFO("*******Plan MoveC State OK");
        }
    } else {
        ROS_ERROR("*******Plan State Fail arrival_flag is %d", arrival_flag);
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
    // moveJ空间规划指令Publisher
    ros::Publisher moveJ_pub = nh.advertise<rm_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);
    // moveJ_P空间规划指令Publisher
    ros::Publisher moveJ_P_pub = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);
    // move_l指令Publisher
    ros::Publisher moveL_pub = nh.advertise<rm_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);
    // move_c指令Publisher
    ros::Publisher moveC_pub = nh.advertise<rm_msgs::MoveC>("/rm_driver/MoveC_Cmd", 10);

    // 订阅机械臂执行状态话题
    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);

    ros::Duration(2.0).sleep();
    /**
     * 2.控制机械臂运动到目标位姿
     */
    //定义一个MoveJ指令的目标位姿
    if(arrival_flag == 0)
    {
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
        ROS_INFO("*******Plan MoveJ Start");
        moveJ_pub.publish(moveJ_BeginPose);
    }
    while((arrival_flag == 0)&&(ros::ok()));
    //延时，确保机械臂执行完上一条轨迹后稳定
    ros::Duration(1.0).sleep();

    if(arrival_flag == 1)
    {
        rm_msgs::MoveJ_P moveJ_P_TargetPose;
        moveJ_P_TargetPose.Pose.position.x = -0.355816;
        moveJ_P_TargetPose.Pose.position.y = -0.000013;
        moveJ_P_TargetPose.Pose.position.z = 0.222814;
        moveJ_P_TargetPose.Pose.orientation.x = 0.995179;
        moveJ_P_TargetPose.Pose.orientation.y = -0.094604;
        moveJ_P_TargetPose.Pose.orientation.z = -0.025721;
        moveJ_P_TargetPose.Pose.orientation.w = 0.002349;
        moveJ_P_TargetPose.trajectory_connect = 0;
        moveJ_P_TargetPose.speed = 0.2;
        // 发布位姿
        ROS_INFO("*******Plan MoveJ_P Start");
        moveJ_P_pub.publish(moveJ_P_TargetPose);
    }
    while((arrival_flag == 1)&&(ros::ok()));
    //延时，确保机械臂执行完上一条轨迹后稳定
    ros::Duration(1.0).sleep();

    if(arrival_flag == 2)
    {
        rm_msgs::MoveL moveL_TargetPose;
        moveL_TargetPose.Pose.position.x = -0.255816;
        moveL_TargetPose.Pose.position.y = -0.000013;
        moveL_TargetPose.Pose.position.z = 0.222814;
        moveL_TargetPose.Pose.orientation.x = 0.995179;
        moveL_TargetPose.Pose.orientation.y = -0.094604;
        moveL_TargetPose.Pose.orientation.z = -0.025721;
        moveL_TargetPose.Pose.orientation.w = 0.002349;
        moveL_TargetPose.speed = 0.6;
        moveL_TargetPose.trajectory_connect = 0;

        // 发布位姿
        ROS_INFO("*******Plan MoveL Start");
        moveL_pub.publish(moveL_TargetPose);
    }
    // 等待到达
    while((arrival_flag == 3)&&(ros::ok()));
    //延时，确保机械臂执行完上一条轨迹后稳定
    ros::Duration(1.0).sleep();

    if((arrival_flag == 3)&&(ros::ok()))
    {
        rm_msgs::MoveC moveC_TargetPose;
        moveC_TargetPose.Mid_Pose.position.x = -0.307239;
        moveC_TargetPose.Mid_Pose.position.y = 0.150903;
        moveC_TargetPose.Mid_Pose.position.z = 0.222814;
        moveC_TargetPose.Mid_Pose.orientation.x = 0.995179;
        moveC_TargetPose.Mid_Pose.orientation.y = -0.094604;
        moveC_TargetPose.Mid_Pose.orientation.z = -0.025721;
        moveC_TargetPose.Mid_Pose.orientation.w = 0.002349;
        moveC_TargetPose.End_Pose.position.x = -0.357239;
        moveC_TargetPose.End_Pose.position.y = 0.000903;
        moveC_TargetPose.End_Pose.position.z = 0.222814;
        moveC_TargetPose.End_Pose.orientation.x = 0.995179;
        moveC_TargetPose.End_Pose.orientation.y = -0.094604;
        moveC_TargetPose.End_Pose.orientation.z = -0.025721;
        moveC_TargetPose.End_Pose.orientation.w = 0.002349;
        moveC_TargetPose.speed = 0.2;
        moveC_TargetPose.loop = 1;
        moveC_TargetPose.trajectory_connect = 0;
        ROS_INFO("*******Plan MoveC Start");
        moveC_pub.publish(moveC_TargetPose);
    }
    // 等待到达
    while((arrival_flag == 4)&&(ros::ok()));

    ros::waitForShutdown();

    return 0;
}

//
// Created by ubuntu on 21-9-14.
//
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <rm_msgs/Gripper_Pick.h>
#include <rm_msgs/Gripper_Set.h>
#include <rm_msgs/MoveL.h>
#include <rm_msgs/MoveC.h>
#include <rm_msgs/MoveJ.h>
#include <rm_msgs/Plan_State.h>
#include <math.h>
//#include <geometry_msgs/PointStamped.h>


// 接收到订阅的消息后，会进入消息回调函数
void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
{
    // 将接收到的消息打印出来
//    ROS_INFO("Marekr pose->position[%f, %f, %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if(msg->state)
    {
        ROS_INFO("*******Plan State OK");
    } else {
        ROS_INFO("*******Plan State Fail");
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_api_movel");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(3);
    spin.start();

    // 初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface group("arm");
    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(0.02);


    /*
     * 1.相关初始化
     */
    //空间规划指令Publisher
    ros::Publisher moveJ_pub = nh.advertise<rm_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);
    //直线规划指令Publisher
    ros::Publisher moveL_pub = nh.advertise<rm_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);



    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);

    ros::Duration(2.0).sleep();

    /**
     * 2.控制机械臂运动到识别抓取的初始位姿
     */
    //定义一个MoveJ指令的起始位姿
    /*
    rm_msgs::MoveJ moveJ_BeginPose;
    moveJ_BeginPose.joint[0] = -0.360829;
    moveJ_BeginPose.joint[1] = 0.528468;
    moveJ_BeginPose.joint[2] = 1.326293;
    moveJ_BeginPose.joint[3] = -0.000454;
    moveJ_BeginPose.joint[4] = 1.221748;
    moveJ_BeginPose.joint[5] = 0.000052;
    moveJ_BeginPose.speed = 0.3;

    //发布空间规划指令运动到起始位姿
    moveJ_pub.publish(moveJ_BeginPose);
    */

    ros::Duration(5.0).sleep();

    moveit::planning_interface::MoveGroupInterface::Plan mPlan;
    moveit::planning_interface::MoveItErrorCode planResult;


    //获取末端当前位姿
    geometry_msgs::PoseStamped currPose_Link6 = group.getCurrentPose("Link6");

    ROS_INFO("currPose_Link6.pose.position(%f, %f, %f)", currPose_Link6.pose.position.x, currPose_Link6.pose.position.y, currPose_Link6.pose.position.z);
    ROS_INFO("currPose_Link6.pose.orientation(%f, %f, %f, %f)", currPose_Link6.pose.orientation.x, currPose_Link6.pose.orientation.y, currPose_Link6.pose.orientation.z, currPose_Link6.pose.orientation.w);

    //Link6目标位姿
    geometry_msgs::Pose targetPose_Link6;
    targetPose_Link6.position.x = currPose_Link6.pose.position.x;
    targetPose_Link6.position.y = currPose_Link6.pose.position.y + 0.04;
    targetPose_Link6.position.z = currPose_Link6.pose.position.z;
    targetPose_Link6.orientation = currPose_Link6.pose.orientation;

    ROS_INFO("targetPose_Link6.position(%f, %f, %f)", targetPose_Link6.position.x, targetPose_Link6.position.y, targetPose_Link6.position.z);
    ROS_INFO("targetPose_Link6.orientation(%f, %f, %f, %f)", targetPose_Link6.orientation.x, targetPose_Link6.orientation.y, targetPose_Link6.orientation.z, targetPose_Link6.orientation.w);


    rm_msgs::MoveL moveL_TargetPose;
    moveL_TargetPose.Pose = targetPose_Link6;
    moveL_TargetPose.speed = 0.2;
    moveL_pub.publish(moveL_TargetPose);


    ros::waitForShutdown();

    return 0;
}


/*
//定义一个MoveJ指令的起始位姿
rm_msgs::MoveJ moveJ_BeginPose;
moveJ_BeginPose.joint[0] = -0.360829;
moveJ_BeginPose.joint[1] = 0.528468;
moveJ_BeginPose.joint[2] = 1.326293;
moveJ_BeginPose.joint[3] = -0.000454;
moveJ_BeginPose.joint[4] = 1.221748;
moveJ_BeginPose.joint[5] = 0.000052;
moveJ_BeginPose.speed = 0.3;

//空间规划指令Publisher
moveJ_pub = nh.advertise<rm_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);

//发布空间规划指令运动到起始位姿
moveJ_pub.publish(moveJ_BeginPose);
*/

//
// Created by ubuntu on 24-7-1.
//
#include <ros/ros.h>
#include <rm_msgs/Arm_Current_State.h>
#include <rm_msgs/ArmState.h>
#include <rm_msgs/GetArmState_Command.h>
#include <rm_msgs/Six_Force.h>
#include <rm_msgs/Arm_Software_Version.h>
#include <std_msgs/Empty.h>

void Get_Arm_Software_version_Callback(const rm_msgs::Arm_Software_Version msg)
{
    ROS_INFO("Product_version is: %s", msg.product_version.c_str());
    ROS_INFO("controller_version is: %s", msg.controller_version.c_str());
    ROS_INFO("algorithm_info is: %s", msg.algorithm_info.c_str());
    ROS_INFO("ctrl_info build_time is: %s", msg.ctrl_info.build_time.c_str());
    ROS_INFO("ctrl_info version is: %s", msg.ctrl_info.version.c_str());
    ROS_INFO("com_info build_time is: %s", msg.com_info.build_time.c_str());
    ROS_INFO("com_info version is: %s", msg.com_info.version.c_str());
    ROS_INFO("program_info build_time is: %s", msg.program_info.build_time.c_str());
    ROS_INFO("program_info version is: %s", msg.program_info.version.c_str());
        
    ROS_INFO("dynamic_info is: %s", msg.dynamic_info.c_str());
    ROS_INFO("plan_info build_time is: %s", msg.plan_info.build_time.c_str());
    ROS_INFO("plan_info version is: %s", msg.plan_info.version.c_str());
}


// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void GetArmState_Callback(const rm_msgs::Arm_Current_State msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg.dof == 7)
    {
        ROS_INFO("joint angle state is: [%f, %f, %f, %f, %f, %f, %f]\n", msg.joint[0],msg.joint[1],msg.joint[2],msg.joint[3],msg.joint[4],msg.joint[5],msg.joint[6]);
    }
    else
    {
        ROS_INFO("joint angle state is: [%f, %f, %f, %f, %f, %f]\n", msg.joint[0],msg.joint[1],msg.joint[2],msg.joint[3],msg.joint[4],msg.joint[5]);
    }

    ROS_INFO("pose Euler angle is: [%f, %f, %f, %f, %f, %f]\n", msg.Pose[0],msg.Pose[1],msg.Pose[2],msg.Pose[3],msg.Pose[4],msg.Pose[5]);
}

// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void Get_Arm_State_Callback(const rm_msgs::ArmState msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg.dof == 7)
    {
        ROS_INFO("joint angle state is: [%f, %f, %f, %f, %f, %f, %f]\n", msg.joint[0],msg.joint[1],msg.joint[2],msg.joint[3],msg.joint[4],msg.joint[5],msg.joint[6]);
    }
    else
    {
        ROS_INFO("joint radian state is: [%f, %f, %f, %f, %f, %f]\n", msg.joint[0],msg.joint[1],msg.joint[2],msg.joint[3],msg.joint[4],msg.joint[5]);
    }
    ROS_INFO("pose Quaternion is: [%f, %f, %f, %f, %f, %f, %f]\n", msg.Pose.position.x,msg.Pose.position.y,msg.Pose.position.z,msg.Pose.orientation.x,msg.Pose.orientation.y,msg.Pose.orientation.z,msg.Pose.orientation.w);
}

void Get_Six_Force_Callback(const rm_msgs::Six_Force msg)
{
    // 原始六维力数据
    ROS_INFO("Six force value is: [force_Fx:%f,\n force_Fy:%f,\n force_Fz:%f,\n force_Mx:%f,\n force_My:%f,\n force_Mz:%f]\n", 
    msg.force_Fx,
    msg.force_Fy,
    msg.force_Fz,
    msg.force_Mx,
    msg.force_My,
    msg.force_Mz);
}

void Get_Six_Zero_Force_Callback(const rm_msgs::Six_Force msg)
{
    // 原始六维力数据
    ROS_INFO("Zero six force value is: [force_Fx:%f,\n force_Fy:%f,\n force_Fz:%f,\n force_Mx:%f,\n force_My:%f,\n force_Mz:%f]\n", 
    msg.force_Fx,
    msg.force_Fy,
    msg.force_Fz,
    msg.force_Mx,
    msg.force_My,
    msg.force_Mz);
}

void Get_Work_Zero_Force_Callback(const rm_msgs::Six_Force msg)
{
    // 原始六维力数据
    ROS_INFO("Work zro force value is: [force_Fx:%f,\n force_Fy:%f,\n force_Fz:%f,\n force_Mx:%f,\n force_My:%f,\n force_Mz:%f]\n", 
    msg.force_Fx,
    msg.force_Fy,
    msg.force_Fz,
    msg.force_Mx,
    msg.force_My,
    msg.force_Mz);
}

void Get_Tool_Zero_Force_Callback(const rm_msgs::Six_Force msg)
{
    // 原始六维力数据
    ROS_INFO("Tool zero force value is: [force_Fx:%f,\n force_Fy:%f,\n force_Fz:%f,\n force_Mx:%f,\n force_My:%f,\n force_Mz:%f]\n", 
    msg.force_Fx,
    msg.force_Fy,
    msg.force_Fz,
    msg.force_Mx,
    msg.force_My,
    msg.force_Mz);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "api_Get_Arm_State_demo");
    ros::NodeHandle nh;

    // 声明spinner对象，参数2表示并发线程数，默认处理全局Callback队列
    ros::AsyncSpinner spin(2);
    // 启动两个spinner线程并发执行可用回调 
    spin.start();

    // 初始化数据
    std_msgs::Empty empty_value;
    rm_msgs::GetArmState_Command command;
    command.command = "get_current_arm_state";

    ros::Duration(1.0).sleep();
    /*
     * 1.相关初始化
     */
    // 获取机械臂当前版本指令
    ros::Publisher test_Get_Arm_Software_Version_pub = nh.advertise<std_msgs::Empty>("/rm_driver/Get_Arm_Software_Version_Cmd", 10);

    // 获取机械臂当前状态指令
    ros::Publisher test_Get_Arm_Base_State_pub = nh.advertise<rm_msgs::GetArmState_Command>("/rm_driver/GetArmState_Cmd", 10);

    // 获取机械臂六维力指令
    ros::Publisher test_Get_Arm_Six_Force_pub = nh.advertise<std_msgs::Empty>("/rm_driver/GetSixForce_Cmd", 10);



    // 等待版本信息更新
    ros::Subscriber Arm_Software_Version_sub = nh.subscribe("/rm_driver/Get_Arm_Software_Version_Result", 10, Get_Arm_Software_version_Callback);
    // 订阅机械臂当前状态指令(角度+欧拉角)
    ros::Subscriber Arm_Base_State_sub = nh.subscribe("/rm_driver/ArmCurrentState", 10, Get_Arm_State_Callback);

    // 订阅机械臂当前状态指令(弧度+四元数)
    ros::Subscriber Arm_New_State_sub = nh.subscribe("/rm_driver/Arm_Current_State", 10, GetArmState_Callback);

    // 订阅机械臂六维力原始数据
    ros::Subscriber GetSixForceState_sub = nh.subscribe("/rm_driver/GetSixForce", 10, Get_Six_Force_Callback);

    // 订阅机械臂传感器坐标系下的六维力数据
    ros::Subscriber SixZeroForceState_sub = nh.subscribe("/rm_driver/SixZeroForce", 10, Get_Six_Zero_Force_Callback);

    // 订阅机械臂工作坐标系下的传感器数据
    ros::Subscriber WorkZeroForceState_sub = nh.subscribe("/rm_driver/WorkZeroForce", 10, Get_Work_Zero_Force_Callback);

    // 订阅机械臂工具坐标系下的传感器数据
    ros::Subscriber ToolZeroForceState_sub = nh.subscribe("/rm_driver/ToolZeroForce", 10, Get_Tool_Zero_Force_Callback);

    ros::Duration(2.0).sleep();

    /**
     * 2.发布指令信息获取机械臂数据
     */
    
    //获取机械臂当前状态指令
    test_Get_Arm_Software_Version_pub.publish(empty_value);
    ROS_INFO("*******Get Arm Software Version Pub show on the rm_driver node.*******");
    ros::Duration(1.0).sleep();
    test_Get_Arm_Base_State_pub.publish(command);
    ROS_INFO("*******Get Arm State Pub*******");
    ros::Duration(1.0).sleep();
    test_Get_Arm_Six_Force_pub.publish(empty_value);
    ROS_INFO("*******Get Arm Six Force Pub*******");
    ros::Duration(1.0).sleep();

    ROS_INFO("*******All command is over please click ctrl+c end*******");
    ros::waitForShutdown();

    return 0;
}

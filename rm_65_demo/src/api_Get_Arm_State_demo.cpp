//
// Created by ubuntu on 22-6-22.
//
#include <ros/ros.h>
#include <rm_msgs/Arm_Current_State.h>
#include <rm_msgs/GetArmState_Command.h>


// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void GetArmState_Callback(const rm_msgs::Arm_Current_State msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    ROS_INFO("joint state is: [%f, %f, %f, %f, %f, %f]\n", msg.joint[0],msg.joint[1],msg.joint[2],msg.joint[3],msg.joint[4],msg.joint[5]);
    ROS_INFO("pose state is: [%f, %f, %f, %f, %f, %f]\n", msg.Pose[0],msg.Pose[1],msg.Pose[2],msg.Pose[3],msg.Pose[4],msg.Pose[5]);
    ROS_INFO("arm_err is:%d\n", msg.arm_err);
    ROS_INFO("sys_err is:%d\n", msg.sys_err);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "api_Get_Arm_State_demo");
    ros::NodeHandle nh;

    // 声明spinner对象，参数2表示并发线程数，默认处理全局Callback队列
    ros::AsyncSpinner spin(2);
    // 启动两个spinner线程并发执行可用回调 
    spin.start();

    ros::Duration(1.0).sleep();
    /*
     * 1.相关初始化
     */
    // 空间规划指令Publisher
    ros::Publisher test_Get_Arm_State_pub = nh.advertise<rm_msgs::GetArmState_Command>("/rm_driver/GetArmState_Cmd", 10);

    // 订阅机械臂执行状态话题
    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Arm_Current_State", 10, GetArmState_Callback);

    ros::Duration(2.0).sleep();

    /**
     * 2.控制机械臂运动到目标位姿
     */
    
    rm_msgs::GetArmState_Command command;
    command.command = "get_current_arm_state";



    //发布空间规划指令使机械臂运动到目标位姿
    test_Get_Arm_State_pub.publish(command);
    ROS_INFO("*******published command:%s", command.command.c_str());

    ros::waitForShutdown();

    return 0;
}

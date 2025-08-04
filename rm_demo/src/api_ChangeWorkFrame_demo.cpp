//
// Created by ubuntu on 22-6-22.
//
#include <ros/ros.h>
#include <rm_msgs/ChangeWorkFrame_Name.h>
#include <rm_msgs/ChangeWorkFrame_State.h>


 
// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void ChangeWorkFrame_Callback(const rm_msgs::ChangeWorkFrame_State::ConstPtr& msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg->state)
    {
        ROS_INFO("*******Switching the work frame succeeded");
    } else {
        ROS_INFO("*******Switching the work frame Failed");
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "api_ChangeWorkFrame_demo");
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
    ros::Publisher changeWorkFrame_pub = nh.advertise<rm_msgs::ChangeWorkFrame_Name>("/rm_driver/ChangeWorkFrame_Cmd", 10);

    // 订阅机械臂执行状态话题
    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/ChangeWorkFrame_State", 10, ChangeWorkFrame_Callback);

    ros::Duration(2.0).sleep();

    /**
     * 2.控制机械臂运动到目标位姿
     */
    
    rm_msgs::ChangeWorkFrame_Name name;
    name.WorkFrame_name = "base";



    //发布空间规划指令使机械臂运动到目标位姿
    changeWorkFrame_pub.publish(name);
    ROS_INFO("*******published tool name:%s", name.WorkFrame_name.c_str());

    ros::waitForShutdown();

    return 0;
}

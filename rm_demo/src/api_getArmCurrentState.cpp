//
// Created by ubuntu on 22-2-22.
//
#include <ros/ros.h>
#include <rm_msgs/GetArmState_Command.h>
#include <std_msgs/Bool.h>


int main(int argc, char **argv)
{
	//ROS节点初始化
	ros::init(argc, argv, "getArmState_publisher");
	//创建节点句柄
	ros::NodeHandle n;

	//创建一个Publisher,发布名为/rm_driver/GetArmState_Cmd的topic,消息类型为rm_msgs::GetArmState_Command,队列长度10
	ros::Publisher pub_getArmState = n.advertise<rm_msgs::GetArmState_Command>("/rm_driver/GetArmState_Cmd", 10);
    ros::Publisher pub_getArmStateTimerSwitch = n.advertise<std_msgs::Bool>("/rm_driver/GetArmStateTimerSwitch", 200);

    ros::Duration(2.0).sleep();

    std_msgs::Bool timerSwitch;
    timerSwitch.data = true;
    pub_getArmStateTimerSwitch.publish(timerSwitch);

	//设置循环的频率
	ros::Rate loop_rate(10);

	int cout = 0;
	while(ros::ok())
	{
		//初始化rm_msgs::GetArmState_Command类型的消息
		rm_msgs::GetArmState_Command msg_getArmState;
        msg_getArmState.command = "get_current_arm_state";


		//发布消息
		pub_getArmState.publish(msg_getArmState);

		//按照循环频率延时
		loop_rate.sleep();
	}
	return 0;
}
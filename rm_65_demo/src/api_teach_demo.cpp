//
// Created by ubuntu on 22-6-22.
//
#include <ros/ros.h>
#include <rm_msgs/Joint_Teach.h>
#include <rm_msgs/Pos_Teach.h>
#include <rm_msgs/Ort_Teach.h>
#include <rm_msgs/Stop_Teach.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "api_teach_demo");
    ros::NodeHandle nh;

    // 声明spinner对象，参数2表示并发线程数，默认处理全局Callback队列
    ros::AsyncSpinner spin(5);
    spin.start();

    ros::Duration(1.0).sleep();
    /*
     * 1.相关初始化
     */
    ros::Publisher test_setJointTeach = nh.advertise<rm_msgs::Joint_Teach>("/rm_driver/Arm_JointTeach", 10);
    ros::Publisher test_setPosTeach = nh.advertise<rm_msgs::Pos_Teach>("/rm_driver/Arm_PosTeach", 10);
    ros::Publisher test_setOrtTeach = nh.advertise<rm_msgs::Ort_Teach>("/rm_driver/Arm_OrtTeach", 10);
    ros::Publisher test_setStopTeach = nh.advertise<rm_msgs::Stop_Teach>("/rm_driver/Arm_StopTeach", 10);

    ros::Duration(2.0).sleep();


    rm_msgs::Joint_Teach Joint_Teach;
    rm_msgs::Pos_Teach Pos_Teach;
    rm_msgs::Ort_Teach Ort_Teach;
    rm_msgs::Stop_Teach Stop_Teach;

    //停止示教
    Stop_Teach.command = "set_stop_teach";

    
    //关节示教
    Joint_Teach.teach_joint = 5;
    Joint_Teach.direction = "pos";
    Joint_Teach.v = 20;


    test_setJointTeach.publish(Joint_Teach);
    ros::Duration(2.0).sleep();
    test_setStopTeach.publish(Stop_Teach);
    ros::Duration(2.0).sleep();

    //关节示教
    Joint_Teach.teach_joint = 5;
    Joint_Teach.direction = "neg";
    Joint_Teach.v = 20;

    test_setJointTeach.publish(Joint_Teach);
    ros::Duration(2.0).sleep();
    test_setStopTeach.publish(Stop_Teach);
    ROS_INFO("joint teach end!");
    ros::Duration(2.0).sleep();
    //*****关节示教结束


    
    //位置示教
    Pos_Teach.teach_type = "z";
    Pos_Teach.direction = "pos";
    Pos_Teach.v = 20;


    test_setPosTeach.publish(Pos_Teach);
    ros::Duration(3.0).sleep();
    test_setStopTeach.publish(Stop_Teach);
    ros::Duration(2.0).sleep();

    //位置示教
    Pos_Teach.teach_type = "z";
    Pos_Teach.direction = "neg";
    Pos_Teach.v = 20;

    test_setPosTeach.publish(Pos_Teach);
    ros::Duration(3.0).sleep();
    test_setStopTeach.publish(Stop_Teach);
    ROS_INFO("pose teach end!");
    ros::Duration(2.0).sleep();
    //*****位置示教结束
    





    //姿态示教
    Ort_Teach.teach_type = "rz";
    Ort_Teach.direction = "pos";
    Ort_Teach.v = 50;


    test_setOrtTeach.publish(Ort_Teach);
    ros::Duration(3.0).sleep();
    test_setStopTeach.publish(Stop_Teach);
    ros::Duration(2.0).sleep();

    //姿态示教
    Ort_Teach.teach_type = "rz";
    Ort_Teach.direction = "neg";
    Ort_Teach.v = 50;

    test_setOrtTeach.publish(Ort_Teach);
    ros::Duration(3.0).sleep();
    test_setStopTeach.publish(Stop_Teach);
    ROS_INFO("ort teach end!");
    ros::Duration(2.0).sleep();
    //*****位姿示教结束


    ros::waitForShutdown();

    return 0;
}


#include "rm_robot.h"
#include <sensor_msgs/JointState.h>

bool joint_flag = false;

void MoveJ_Callback(const rm_msgs::MoveJ msg)
{
    int res = 0;
    int i = 0;
    byte speed;
    float joint[6];

    speed = (byte)(msg.speed*100);
    for(i=0;i<6;i++)
    {
        joint[i] = msg.joint[i]*RAD_DEGREE;
    }

    res = Movej_Cmd(joint, speed);
    joint_flag = true;
    if(res == 0)
    {
        ROS_INFO("MoveJ success!\n");
    }
    else
    {
        ROS_ERROR("MoveJ failed!\n");
    }
}
void MoveL_Callback(const rm_msgs::MoveL msg)
{
    int res = 0;
    byte speed;
    POSE target;

    target = Quater_To_Euler(msg.Pose);
    speed = (byte)(msg.speed*100);

    res = Movel_Cmd(target, speed);
    joint_flag = true;
    if(res == 0)
    {
        ROS_INFO("MoveL success!\n");
    }
    else
    {
        ROS_ERROR("MoveL failed!\n");
    }

}
void MoveC_Callback(const rm_msgs::MoveC msg)
{
    int res = 0;
    byte speed;
    POSE target1, target2;

    target1 = Quater_To_Euler(msg.Mid_Pose);
    target2 = Quater_To_Euler(msg.End_Pose);
    speed = (byte)(msg.speed*100);

    res = Movec_Cmd(target1, target2, speed);
    joint_flag = true;
    if(res == 0)
    {
        ROS_INFO("MoveC success!\n");
    }
    else
    {
        ROS_ERROR("MoveC failed!\n");
    }
}
void JointPos_Callback(const rm_msgs::JointPos msg)
{
    int res = 0;
    int i = 0;
    float joint[6];

    for(i=0;i<6;i++)
    {
        joint[i] = msg.joint[i]*RAD_DEGREE;
    }

    res = Movej_CANFD(joint);
    joint_flag = true;
    if(res == 0)
    {
        //ROS_INFO("JointPos success!\n");
    }
    else
    {
        ROS_ERROR("JointPos failed!\n");
    }
}
void Arm_DO_Callback(const rm_msgs::Arm_Digital_Output msg)
{
    int res = 0;
    if((msg.num > 0) && (msg.num <5))
    {
        res = Set_DO_State(msg.num, msg.state);
        if(res == 0)
        {
            ROS_INFO("Arm Digital IO set success!\n");
        }
        else
        {
            ROS_ERROR("Arm Digital IO set failed!\n");
        }
    }
    else
    {
        ROS_ERROR("Arm digital IO num wrong!\n");
    }
}
void Arm_AO_Callback(const rm_msgs::Arm_Analog_Output msg)
{
    int res = 0;
    if((msg.num > 0) && (msg.num <5))
    {
        res = Set_AO_State(msg.num, msg.voltage);
        if(res == 0)
        {
            ROS_INFO("Arm analog IO set success!\n");
        }
        else
        {
            ROS_ERROR("Arm analog IO set failed!\n");
        }
    }
    else
    {
        ROS_ERROR("Arm Analog IO num wrong!\n");
    }
}
void Tool_DO_Callback(const rm_msgs::Tool_Digital_Output msg)
{
    int res = 0;
    if((msg.num > 0) && (msg.num <3))
    {
        res = Set_Tool_DO_State(msg.num, msg.state);
        if(res == 0)
        {
            ROS_INFO("Tool Digital IO set success!\n");
        }
        else
        {
            ROS_ERROR("Tool Digital IO set failed!\n");
        }
    }
    else
    {
        ROS_ERROR("Tool digital IO num wrong!\n");
    }
}
void Tool_AO_Callback(const rm_msgs::Tool_Analog_Output msg)
{
    int res = 0;

    res = Set_Tool_AO_State(msg.voltage);
    if(res == 0)
    {
        ROS_INFO("Tool analog IO set success!\n");
    }
    else
    {
        ROS_ERROR("Tool analog IO set failed!\n");
    }
}
void Gripper_Pick_Callback(rm_msgs::Gripper_Pick msg)
{
    int res = 0;

    if(msg.speed < 1)
    {
        msg.speed = 1;
    }
    else if(msg.speed > 1000)
    {
        msg.speed = 1000;
    }
    if(msg.force < 1)
    {
        msg.force = 1;
    }
    else if(msg.force > 1000)
    {
        msg.force = 1000;
    }

    res = Set_Gripper_Pick(msg.speed, msg.force);
    if(res == 0)
    {
        if(res == 0)
        {
            RM_Joint.gripper_joint = 0;
            ROS_INFO("Gripper pick success!\n");
        }
        else
        {
            ROS_ERROR("Gripper pick failed!\n");
        }
    }
}

void Gripper_Set_Callback(rm_msgs::Gripper_Set msg)
{
    int res = 0;

    if(msg.position < 1)
    {
        msg.position = 1;
    }
    else if(msg.position > 1000)
    {
        msg.position = 1000;
    }

    res = Set_Gripper(msg.position);
    if(res == 0)
    {
        if(res == 0)
        {
            RM_Joint.gripper_joint = GRIPPER_WIDTH*msg.position/2000;
            ROS_INFO("Gripper set success!\n");
        }
        else
        {
            ROS_ERROR("Gripper set failed!\n");
        }
    }
}
void Stop_Callback(const rm_msgs::Stop msg)
{

    int res = 0;

    if(msg.state)
    {
        res = Move_Stop_Cmd();
        if(res == 0)
        {
            ROS_INFO("Emergency stop success!\n");
        }
        else
        {
            ROS_ERROR("Emergency stop failed!\n");
        }
    }
}

void Joint_Enable_Callback(const rm_msgs::Joint_Enable msg)
{
    int res = 0;
    if((msg.joint_num > 6) || (msg.joint_num < 1))
    {
        ROS_ERROR("Joint Enable Set Error:Joint num out of range");
        return;
    }
    //Enable Joint, Firstly, clear joint err
    if(msg.state)
    {
        res = Clear_Joint_Err(msg.joint_num);
        if(res != 0)
        {
            ROS_ERROR("Joint Enable Set Failed");
            return;
        }
    }
    res = Set_Joint_Enable(msg.joint_num, msg.state);
    if(res != 0)
    {
        ROS_ERROR("Joint Enable Set Failed");
        return;
    }

}

void  IO_Update_Callback(const rm_msgs::IO_Update msg)
{
    int res = 0;
    //Arm IO Update
    if(msg.type == 1)
    {
       res = Get_IO_Input();
    }
    // Tool IO Update
    else if(msg.type == 2)
    {
        res = Get_Tool_IO_Input();
    }
    if(res != 0)
    {
        ROS_ERROR("Joint Enable Set Failed");
        return;
    }
}

void timer_callback(const ros::TimerEvent)
{

    timer_cnt++;

    if(joint_flag)
    {
        joint_flag = false;
        return;
    }
    if(timer_cnt > 1)
    {
        timer_cnt = 0;
        Get_Joint_Err_Flag();
    }
    else
    {
        Get_Arm_Joint();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh_;
    ros::Rate loop_rate(200);      //200Hz,5ms
    int cnt = 0, i = 0;
    struct timeval time_out;
    time_out.tv_sec = 0;
    time_out.tv_usec = 0;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(Arm_Socket, &fds);
    int nRet;
    byte temp[2], msg;
    int res = 0;

    memset(temp, 0, sizeof(temp));

    char socket_buffer[100];
    memset(socket_buffer, 0, sizeof(socket_buffer));
    int buffer_cnt = 0;

    //msg
    rm_msgs::Arm_IO_State Arm_IO;
    rm_msgs::Tool_IO_State Tool_IO;
    rm_msgs::Plan_State Plan;

    //subscriber
    MoveJ_Cmd = nh_.subscribe("/rm_driver/MoveJ_Cmd", 10, MoveJ_Callback);
    MoveL_Cmd = nh_.subscribe("/rm_driver/MoveL_Cmd", 10, MoveL_Callback);
    MoveC_Cmd = nh_.subscribe("/rm_driver/MoveC_Cmd", 10, MoveC_Callback);
    JointPos_Cmd = nh_.subscribe("/rm_driver/JointPos", 10, JointPos_Callback);
    Arm_DO_Cmd = nh_.subscribe("/rm_driver/Arm_Digital_Output", 10, Arm_DO_Callback);
    Arm_AO_Cmd = nh_.subscribe("/rm_driver/Arm_Analog_Output", 10, Arm_AO_Callback);
    Tool_DO_Cmd = nh_.subscribe("/rm_driver/Tool_Digital_Output", 10, Tool_DO_Callback);
    Tool_AO_Cmd = nh_.subscribe("/rm_driver/Tool_Analog_Output", 10, Tool_AO_Callback);
    Gripper_Cmd = nh_.subscribe("/rm_driver/Gripper_Pick", 10, Gripper_Pick_Callback);
    Gripper_Set_Cmd = nh_.subscribe("/rm_driver/Gripper_Set", 10, Gripper_Set_Callback);
    Emergency_Stop = nh_.subscribe("/rm_driver/Emergency_Stop", 1, Stop_Callback);
    Joint_En = nh_.subscribe("/rm_driver/Joint_Enable", 10, Joint_Enable_Callback);
    IO_Update = nh_.subscribe("/rm_driver/IO_Update", 1, IO_Update_Callback);
    //publisher
    Joint_State = nh_.advertise<sensor_msgs::JointState>("joint_states", 300);
    Arm_IO_State = nh_.advertise<rm_msgs::Arm_IO_State>("/rm_driver/Arm_IO_State", 1);
    Tool_IO_State = nh_.advertise<rm_msgs::Tool_IO_State>("/rm_driver/Tool_IO_State", 1);
    Plan_State = nh_.advertise<rm_msgs::Plan_State>("/rm_driver/Plan_State", 1);

    //timer
    State_Timer = nh_.createTimer(ros::Duration(0.2), timer_callback);

    //init gripper
    RM_Joint.gripper_joint = GRIPPER_WIDTH/2;

    sensor_msgs::JointState real_joint;
    //发送规划角度，仿真真实机械臂连不上
    // real_joint.name.resize(7);
    // real_joint.position.resize(7);
    real_joint.name.resize(6);
    real_joint.position.resize(6);
    real_joint.name[0] = "joint1";
    real_joint.name[1] = "joint2";
    real_joint.name[2] = "joint3";
    real_joint.name[3] = "joint4";
    real_joint.name[4] = "joint5";
    real_joint.name[5] = "joint6";
    // real_joint.name[6] = "gripper_joint";


    while(Arm_Socket_Start())
    {
        cnt++;
        if(cnt > 5)
        {
            cnt = 0;
            ROS_ERROR("/****************************************************************************\\n");
            ROS_ERROR("/**********************Cannot connect RM-65 robot!***************************\\n");
            ROS_ERROR("/****************************************************************************\\n");
            return -1;
        }
        usleep(1000);
    }
    ROS_INFO("/****************************************************************************\\n");
    ROS_INFO("\t\t   Connect RM-65 robot! \t\t   \n");
    ROS_INFO("/****************************************************************************\\n");
    timer_cnt = 0;

    //get robot state
    Get_Arm_Joint();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok())
    {
        while(1)
        {
            FD_ZERO(&fds);
            FD_SET(Arm_Socket, &fds);
            nRet = select(FD_SETSIZE, &fds, NULL, NULL, &time_out);
            if(nRet == 0)
            {
                break;
            }
            recv(Arm_Socket, temp, 1, 0);
            socket_buffer[buffer_cnt] = (char)temp[0];
            buffer_cnt++;
            if((temp[0] == 0x0A) && (buffer_cnt > 1))
            {
                msg = (byte)socket_buffer[buffer_cnt-2];
                if(msg == 0x0D)
                {
                    res = Parser_Msg(socket_buffer);
                    switch(res)
                    {
                        case ARM_JOINT_STATE:
                            real_joint.header.stamp = ros::Time::now();
                            for(i=0;i<6;i++)
                            {
                                real_joint.position[i] = RM_Joint.joint[i]*DEGREE_RAD;
                                // ROS_INFO("joint[%d].position=%f",  i, real_joint.position[i]);
                            }
                            // real_joint.position[6] = RM_Joint.gripper_joint;
                            Joint_State.publish(real_joint);
                            break;
                        case ARM_JOINT_ERR:
                            Info_Joint_Err();
                            break;
                        case ARM_IO_INPUT:
                            for(i=0;i<3;i++)
                            {
                                Arm_IO.Arm_Analog_Input[i] = RM_Joint.Arm_AI[i];
                                Arm_IO.Arm_Digital_Input[i] = RM_Joint.Arm_DI[i];
                            }
                            Arm_IO.Arm_Analog_Input[3] = RM_Joint.Arm_AI[3];
                            Arm_IO_State.publish(Arm_IO);
                            break;
                        case TOOL_IO_INPUT:
                            for(i=0;i<2;i++)
                            {
                                Tool_IO.Tool_Digital_Input[i] = RM_Joint.Tool_DI[i];
                            }
                            Tool_IO.Tool_Analog_Input = RM_Joint.Tool_AI;
                            Tool_IO_State.publish(Tool_IO);
                            break;
                        case PLAN_STATE_TYPE:
                            Plan.state = RM_Joint.plan_flag;
                            Plan_State.publish(Plan);
                            if(Plan.state == 0x00)
                            {
                                ROS_ERROR("Real Arm Trajectory Planning Error!");
                            }
                            break;
                        default:
                            break;
                    }
                    buffer_cnt = 0;
                    memset(socket_buffer, 0, sizeof(socket_buffer));
                    break;
                }
            }
            else if(buffer_cnt > 100)
            {
                buffer_cnt = 0;
                memset(socket_buffer, 0, sizeof(socket_buffer));
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    Arm_Socket_Close();
    ROS_INFO("RM_Robot driver shut down!\n");
    ros::waitForShutdown();
}

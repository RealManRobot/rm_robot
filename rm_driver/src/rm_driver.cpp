#include "rm_robot.h"
#include <chrono>

ros::CallbackQueue queue_others;
ros::CallbackQueue queue_armJog;
ros::CallbackQueue queue_forcePositionMove;
// ros::CallbackQueue queue_follow;

bool joint_flag = false;


void SetToolVoltage_Callback(const std_msgs::Byte msg)
{
    int res = 0;
    byte state;
    state = msg.data;

    res = Set_Tool_Voltage_Cmd(state);
    if (res == 0)
    {
        ROS_INFO("SetToolVoltage success!\n");
    }
    else
    {
        ROS_ERROR("SetToolVoltage failed!\n");
    }
}

void SetHandPosture_Callback(const rm_msgs::Hand_Posture msg)
{
    ROS_INFO("SetHandPosture_Callback!\n");
    uint16_t posture_num = msg.posture_num;
    ROS_INFO("recv SetHandPosture message[posture_num:%d]\r\n", posture_num);
    int res = 0;
    res = SetHandPostureCmd(posture_num,msg.block);
    if (res == 0)
    {
        ROS_INFO("send SetHandPosture cmd success!\n");
    }
    else
    {
        ROS_ERROR("send SetHandPosture cmd failed!\n");
    }
}

//订阅对设置灵巧手动作序列的主题接收到数据后的处理函数
void SetHandSeq_Callback(const rm_msgs::Hand_Seq msg)
{
    ROS_INFO("SetHandSeq_Callback!\n");
    uint16_t seq_num = msg.seq_num;
    ROS_INFO("recv SetHandSeq message[seq_num:%d]\r\n", seq_num);
    int res = 0;
    res = SetHandSeqCmd(seq_num,msg.block);
    if (res == 0)
    {
        ROS_INFO("send SetHandSeq cmd success!\n");
    }
    else
    {
        ROS_ERROR("send SetHandSeq cmd failed!\n");
    }
}

//订阅对设置灵巧手关节速度的主题接收到数据后的处理函数
void SetHandSpeed_Callback(const rm_msgs::Hand_Speed msg)
{
    ROS_INFO("SetHandSpeed_Callback!\n");
    uint16_t hand_speed = msg.hand_speed;
    ROS_INFO("recv SetHandSpeed message[hand_speed:%d]\r\n", hand_speed);
    int res = 0;
    res = SetHandSpeedCmd(hand_speed);
    if (res == 0)
    {
        ROS_INFO("send SetHandSpeed cmd success!\n");
    }
    else
    {
        ROS_ERROR("send SetHandSpeed cmd failed!\n");
    }
}

void HandFollowPos_Callback(const rm_msgs::Hand_Angle msg)
{
    int res = 0;
    int i = 0;
    int16_t hand_angle[6];

    for (i = 0; i < 6; i++)
    {
        hand_angle[i] = msg.hand_angle[i];
    }

    res = SetHandFollowPos(hand_angle);

    if (res != 0)
    {
        ROS_ERROR("SetHandAngle failed!\n");
    }
}

void HandFollowAngle_Callback(const rm_msgs::Hand_Angle msg)
{
    int res = 0;
    int i = 0;
    int16_t hand_angle[6];

    for (i = 0; i < 6; i++)
    {
        hand_angle[i] = msg.hand_angle[i];
    }

    res = SetHandFollowAngle(hand_angle);

    if (res != 0)
    {
        ROS_ERROR("SetHandAngle failed!\n");
    }
}

//订阅对设置灵巧手关节力阈值的主题接收到数据后的处理函数
void SetHandForce_Callback(const rm_msgs::Hand_Force msg)
{
    ROS_INFO("SetHandForce_Callback!\n");
    uint16_t hand_force = msg.hand_force;
    ROS_INFO("recv SetHandForce message[hand_force:%d]\r\n", hand_force);
    int res = 0;
    res = SetHandForceCmd(hand_force);
    if (res == 0)
    {
        ROS_INFO("send SetHandForce cmd success!\n");
    }
    else
    {
        ROS_ERROR("send SetHandForce cmd failed!\n");
    }
}

//订阅对设置灵巧手角度的主题接收到数据后的处理函数
void SetHandAngle_Callback(const rm_msgs::Hand_Angle msg)
{
    int res = 0;
    int i = 0;
    int16_t hand_angle[6];

    for (i = 0; i < 6; i++)
    {
        hand_angle[i] = msg.hand_angle[i];
    }

    res = SetHandAngle(hand_angle,msg.block);

    if (res == 0)
    {
        ROS_INFO("SetHandAngle success!\n");
    }
    else
    {
        ROS_ERROR("SetHandAngle failed!\n");
    }
}

void SetArmPower_Callback(const std_msgs::Byte msg)
{
    int res = 0;
    byte state;
    state = msg.data;

    res = Set_Arm_Power_Cmd(state);
    if (res == 0)
    {
        ROS_INFO("SetArmPower success!\n");
    }
    else
    {
        ROS_ERROR("SetArmPower failed!\n");
    }
}

void SetJointStep_Callback(const rm_msgs::Joint_Step msg)
{
    int res = 0;
    uint8_t num;
    float angle;
    byte speed;

    num = msg.joint_num;
    angle = msg.step_angle;
    speed = (byte)(msg.speed * 100);

    res = Set_Joint_Step(num, angle, speed);
    if (res == 0)
    {
        ROS_INFO("SetJointStep success!\n");
    }
    else
    {
        ROS_ERROR("SetJointStep failed!\n");
    }
}

// Test
void GetTotalWorkFrame_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("GetTotalWorkFrame_Callback!\n");
    int res = 0;
    res = Get_Total_Work_Frame();
    if (res != 0)
    {
        ROS_ERROR("send GetTotalWorkFrame cmd failed!\n");
    }
}

//查询关节当前电流话题回调
void GetCurrJointCurrent_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("GetCurrJointCurrent_Callback!\n");
    int res = 0;
    res = Get_Current_Joint_Current();
    if (res != 0)
    {
        ROS_ERROR("send GetCurrentJointCurrent cmd failed!\n");
    }
}

void GetLiftState_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = Get_Lift_State();
    if (res != 0)
    {
        ROS_ERROR("send GetLiftState cmd failed!\n");
    }
}

void LiftHeightCtr_Callback(const rm_msgs::Lift_Height msg)
{
    ROS_INFO("LiftHeightCtr_Callback!\n");
    int16_t height = msg.height;
    int16_t speed = msg.speed;
    ROS_INFO("recv lift control message[height:%d speed:%d]\r\n", height, speed);
    int res = 0;
    res = Lift_SetHeightCmd(height, speed);
    if (res == 0)
    {
        ROS_INFO("send lift control cmd success!\n");
    }
    else
    {
        ROS_ERROR("send lift control cmd failed!\n");
    }
}

void LiftSpeedCtr_Callback(const rm_msgs::Lift_Speed msg)
{
    ROS_INFO("LiftSpeedCtr_Callback!\n");
    int16_t speed = msg.speed;
    ROS_INFO("recv lift speed control message[speed:%d]\r\n", speed);
    int res = 0;
    res = SetLiftSpeedCmd(speed);
    if (res == 0)
    {
        ROS_INFO("send lift speed control cmd success!\n");
    }
    else
    {
        ROS_ERROR("send lift speed control cmd failed!\n");
    }
}

void GetArmJoint_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("GetArmJoint_Callback!\n");

    int res = 0;
    res = Get_Arm_Joint();
    if (res != 0)
    {
        ROS_ERROR("send GetArmJoint cmd failed!\n");
    }
}

void MoveJ_Callback(const rm_msgs::MoveJ msg)
{
    int res = 0;
    int i = 0;
    u_int8_t trajectory_connect = msg.trajectory_connect;
    byte speed;
    float joint[7];

    speed = (byte)(msg.speed * 100);
    for (i = 0; i < 6; i++)
    {
        joint[i] = msg.joint[i] * RAD_DEGREE;
    }
    if(arm_dof == 7)
    {
        joint[6] = msg.joint[6] * RAD_DEGREE;
    }

    res = Movej_Cmd(joint, speed, trajectory_connect);
    joint_flag = true;
    if (res == 0)
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
    u_int8_t trajectory_connect = msg.trajectory_connect;
    byte speed;
    POSE target;

    target = Quater_To_Euler(msg.Pose);
    speed = (byte)(msg.speed * 100);

    res = Movel_Cmd(target, speed, trajectory_connect);
    joint_flag = true;
    if (res == 0)
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
    u_int8_t trajectory_connect = msg.trajectory_connect;
    uint16_t speed;
    POSE target1, target2;
    uint16_t loop;

    target1 = Quater_To_Euler(msg.Mid_Pose);
    target2 = Quater_To_Euler(msg.End_Pose);
    speed = (uint16_t)(msg.speed * 100);
    loop = msg.loop;
    res = Movec_Cmd(target1, target2, speed, loop, trajectory_connect);
    joint_flag = true;
    if (res == 0)
    {
        ROS_INFO("MoveC success!\n");
    }
    else
    {
        ROS_ERROR("MoveC failed!\n");
    }
}

/***** ********************************START**************************************
 * 20211103修改: 增加对机械臂的关节示教,位置示教,姿态示教和停止示教
 * *********************************************************************************/
//订阅对机械臂关节示教的主题接收到数据后的处理函数
void JointTeach_Callback(const rm_msgs::Joint_Teach msg)
{
    ROS_INFO("JointTeach_Callback!\n");
    int16_t teach_joint = msg.teach_joint;
    std::string direction = msg.direction;
    int16_t v = msg.v;

    ROS_INFO("recv joint teach message[teach_joint:%d   direction:%s   v:%d]\r\n", teach_joint, direction.c_str(), v);
    int res = 0;
    res = SetJointTeachCmd(teach_joint, direction, v);
    if (res == 0)
    {
        ROS_INFO("send joint teach cmd success!\n");
    }
    else
    {
        ROS_ERROR("send joint teach cmd failed!\n");
    }
}

//订阅对机械臂位置示教的主题接收到数据后的处理函数
void PosTeach_Callback(const rm_msgs::Pos_Teach msg)
{
    ROS_INFO("PosTeach_Callback!\n");
    std::string teach_type = msg.teach_type;
    std::string direction = msg.direction;
    int16_t v = msg.v;

    ROS_INFO("recv position teach message[teach_type:%s   direction:%s   v:%d]\r\n", teach_type.c_str(), direction.c_str(), v);
    int res = 0;
    res = SetPosTeachCmd(teach_type, direction, v);
    if (res == 0)
    {
        ROS_INFO("send position teach cmd success!\n");
    }
    else
    {
        ROS_ERROR("send position teach cmd failed!\n");
    }
}

//订阅对机械臂姿态示教的主题接收到数据后的处理函数
void OrtTeach_Callback(const rm_msgs::Ort_Teach msg)
{
    ROS_INFO("OrtTeach_Callback!\n");
    std::string teach_type = msg.teach_type;
    std::string direction = msg.direction;
    int16_t v = msg.v;

    ROS_INFO("recv ort teach message[teach_type:%s   direction:%s   v:%d]\r\n", teach_type.c_str(), direction.c_str(), v);
    int res = 0;
    res = SetOrtTeachCmd(teach_type, direction, v);
    if (res == 0)
    {
        ROS_INFO("send ort teach cmd success!\n");
    }
    else
    {
        ROS_ERROR("send ort teach cmd failed!\n");
    }
}

//订阅对机械臂停止示教的主题接收到数据后的处理函数
void StopTeach_Callback(const rm_msgs::Stop_Teach msg)
{
    ROS_INFO("StopTeach_Callback!\n");

    int res = 0;
    res = SetStopTeachCmd();
    if (res == 0)
    {
        ROS_INFO("send stop teach cmd success!\n");
    }
    else
    {
        ROS_ERROR("send stop teach cmd failed!\n");
    }
}
/***** ********************************END****************************************/

/***** ********************************START**************************************
 * 20220628修改: 增加MoveJ_P指令、对机械臂切换工具坐标系、切换工作坐标系、查询机械臂状态
 * *********************************************************************************/
void MoveJ_P_Callback(const rm_msgs::MoveJ_P msg)
{
    int res = 0;
    uint8_t trajectory_connect = msg.trajectory_connect;
    byte speed;
    POSE target;

    target = Quater_To_Euler(msg.Pose);
    speed = (byte)(msg.speed * 100);

    res = Movej_p_Cmd(target, speed, trajectory_connect);
    joint_flag = true;
    if (res == 0)
    {
        ROS_INFO("MoveJ_P success!\n");
    }
    else
    {
        ROS_ERROR("MoveJ_P failed!\n");
    }
}
//订阅切换机械臂当前工具坐标系的话题，接收到消息后的回调函数
void ChangeToolName_Callback(const rm_msgs::ChangeTool_Name msg)
{
    int res = 0;
    std::string toolname = msg.toolname;

    res = ChangeToolName_Cmd(toolname);

    if (res == 0)
    {
        // ROS_INFO("Sending command succced!\n");
    }
    else
    {
        ROS_ERROR("Tool coordinate system switching failed!\n");
    }
}
//订阅切换机械臂当前工作坐标系的话题，接收到消息后的回调函数
void ChangeWorkFrame_Callback(const rm_msgs::ChangeWorkFrame_Name msg)
{
    int res = 0;
    std::string WorkFrame_name = msg.WorkFrame_name;

    res = ChangeWorkFrame_Cmd(WorkFrame_name);

    if (res == 0)
    {
        // ROS_INFO("Sending command succced!\n");
    }
    else
    {
        ROS_ERROR("Work Frame switching failed!\n");
    }
}
//订阅查询机械臂当前状态的话题，接收到消息后的回调函数
void GetArmState_Callback(const rm_msgs::GetArmState_Command msg)
{
    int res = 0;
    std::string GetArmState_command = msg.command;

    res = GetArmState_Cmd(GetArmState_command);

    if (res == 0)
    {
        ROS_INFO("Sending command succced!\n");
    }
    else
    {
        ROS_ERROR("Get arm current state failed!\n");
    }
}

//订阅查询机械臂当前状态的话题，接收到消息后的回调函数
void GetCurrentArmState_Callback(const std_msgs::Empty msg)
{
    int res = 0;

    res = GetCurrentArmState_Cmd();

    if (res == 0)
    {
        ROS_INFO("send getArmCurrentState command succced!\n");
    }
    else
    {
        ROS_ERROR("Get arm current state failed!\n");
    }
}
/*************************************END****************************************/

/***** ********************************START***************************************
 * 20220808修改: 增加对机械臂的复合拖动示教、力位混合控制、结束力位混合控制
 开启透传力位混合控制补偿模式、透传力位混合补偿（角度）、透传力位混合补偿（位姿）、关闭透传力位混合控制补偿模式
 * *********************************************************************************/
//开始复合模式拖动示教
// old version
// void StartMultiDragTeach_Callback(const rm_msgs::Start_Multi_Drag_Teach msg)
// {
//     startMulitiDragTeach = true;
//     int res = 0;
//     if ((msg.mode >= 0) && (msg.mode < 4))
//     {
//         res = Start_Multi_Drag_Teach_Cmd(msg.mode);
//         if (res == 0)
//         {
//             ROS_INFO("Start Multi Drag Teach success!\n");
//         }
//         else
//         {
//             ROS_ERROR("Start Multi Drag Teach failed!\n");
//         }
//     }
//     else
//     {
//         ROS_ERROR("Multi Drag Teach mode wrong!\n");
//     }
// }
// new version
void StartMultiDragTeach_Callback(const rm_msgs::Start_Multi_Drag_Teach msg)
{
    startMulitiDragTeach = true;
    int res = 0;
    int free_axes[6];
    for(int i=0;i<6;i++){
        free_axes[i] = msg.free_axes[i];
    }
    res = Start_Multi_Drag_Teach_Cmd(free_axes,msg.frame,msg.singular_wall);
    if (res == 0)
    {
        ROS_INFO("Start Multi Drag Teach success!\n");
    }
    else
    {   
        ROS_ERROR("Start Multi Drag Teach failed!\n");
    }
}

//拖动示教停止
void StopDragTeach_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("StopDragTeach_Callback!\n");
    startMulitiDragTeach = false;
    int res = 0;
    res = Stop_Drag_Teach_Cmd();
    if (res != 0)
    {
        ROS_ERROR("send StopDragTeach cmd failed!\n");
    }
}

//力位混合控制
void SetForcePosition_Callback(const rm_msgs::Set_Force_Position msg)
{
    int res = 0;
    if ((msg.mode >= 0) && (msg.mode < 2))                     //Mode：0-工作坐标系力控；1-工具坐标系力控；
    {
        if ((msg.sensor >= 0) && (msg.sensor < 2))             //Sensor:传感器；0-一维力；1-六维力 
        {
            if ((msg.direction >= 0) && (msg.direction < 6))   //direction：力控方向；0-沿X轴；1-沿Y轴；2-沿Z轴；3-沿RX姿态方向；4-沿RY姿态方向；5-沿RZ姿态方向
            {
                res = Set_Force_Position_Cmd(msg.mode, msg.sensor, msg.N, msg.direction);
                if (res == 0)
                {
                    ROS_INFO("Set Force Position success!\n");
                }
                else
                {
                    ROS_ERROR("Set Force Position failed!\n");
                }
            }
            else
            {
                ROS_ERROR("Set Force Position load wrong!\n");
            }
        }
        else
        {
            ROS_ERROR("Set Force Position sensor wrong!\n");
        }
    }
    else
    {
        ROS_ERROR("Set Force Position mode wrong!\n");
    }
}

//结束力位混合控制
void StopForcePosition_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = Stop_Force_Position_Cmd();
    if (res == 0)
    {
        ROS_INFO("Stop Force Position success!\n");
    }
    else
    {
        ROS_ERROR("Stop Force Position failed!\n");
    }
}
//开启透传力位混合控制补偿模式
void StartForcePositionMove_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = Start_Force_Position_Move_Cmd();
    if (res == 0)
    {
        ROS_INFO("Start Force Position Move success!\n");
    }
    else
    {
        ROS_ERROR("Start Force Position Move failed!\n");
    }
}

//停止透传力位混合控制补偿模式
void StopForcePositionMove_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = Stop_Force_Position_Move_Cmd();
    if (res == 0)
    {
        ROS_INFO("Stop Force Position Move success!\n");
    }
    else
    {
        ROS_ERROR("Stop Force Position Move failed!\n");
    }
}

//位姿透传力位混合补偿
void ForcePositionMovePose_Callback(const rm_msgs::Force_Position_Move_Pose msg)
{
    int res = 0;
    POSE target;
    target = Quater_To_Euler(msg.Pose);

    if ((msg.mode >= 0) && (msg.mode < 2))
    {
        if ((msg.sensor >= 0) && (msg.sensor < 2))
        {
            if ((msg.dir >= 0) && (msg.dir < 6))
            {
                res = Force_Position_Move_Pose_Cmd(msg.mode, msg.sensor, msg.dir, msg.force, canfd_follow, trajectory_mode_, radio_, target);
                if (res == 0)
                {
                    ROS_INFO("Force Position Move Pose success!\n");
                }
                else
                {
                    ROS_ERROR("Force Position Move Pose failed!\n");
                }
            }
            else
            {
                ROS_ERROR("Force Position Move Pose dir wrong!\n");
            }
        }
        else
        {
            ROS_ERROR("Force Position Move Pose sensor wrong!\n");
        }
    }
    else
    {
        ROS_ERROR("Force Position Move Pose mode wrong!\n");
    }
}
//自定义模式位姿透传力位混合补偿
void ForcePositionMovePoseCustom_Callback(const rm_msgs::Force_Position_Move_Pose_Custom msg)
{
    int res = 0;
    POSE target;
    target = Quater_To_Euler(msg.Pose);
    if ((msg.mode >= 0) && (msg.mode < 2))
    {
        if ((msg.sensor >= 0) && (msg.sensor < 2))
        {
            if ((msg.dir >= 0) && (msg.dir < 6))
            {
                res = Force_Position_Move_Pose_Cmd(msg.mode, msg.sensor, msg.dir, msg.force, msg.follow, msg.trajectory_mode, msg.radio, target);
                if (res == 0)
                {
                    ROS_INFO("Force Position Move Pose success!\n");
                }
                else
                {
                    ROS_ERROR("Force Position Move Pose failed!\n");
                }
            }
            else
            {
                ROS_ERROR("Force Position Move Pose dir wrong!\n");
            }
        }
        else
        {
            ROS_ERROR("Force Position Move Pose sensor wrong!\n");
        }
    }
    else
    {
        ROS_ERROR("Force Position Move Pose mode wrong!\n");
    }
}
//角度透传力位混合补偿
void ForcePositionMoveJiont_Callback(const rm_msgs::Force_Position_Move_Joint msg)
{
    int res = 0;
    float joint[7];

    for (int i = 0; i < 6; i++)
    {
        joint[i] = msg.joint[i] * RAD_DEGREE;
    }
    if(arm_dof == 7)
    {
        joint[6] = msg.joint[6] * RAD_DEGREE;
    }

    if ((msg.mode >= 0) && (msg.mode < 2))
    {
        if ((msg.sensor >= 0) && (msg.sensor < 2))
        {
            if ((msg.dir >= 0) && (msg.dir < 6))
            {
                res = Force_Position_Move_Jiont_Cmd(msg.mode, msg.sensor, msg.dir, msg.force, joint);
                if (res == 0)
                {
                    ROS_INFO("Force Position Move Jiont success!\n");
                }
                else
                {
                    ROS_ERROR("Force Position Move Jiont failed!\n");
                }
            }
            else
            {
                ROS_ERROR("Force Position Move Jiont dir wrong!\n");
            }
        }
        else
        {
            ROS_ERROR("Force Position Move Jiont sensor wrong!\n");
        }
    }
    else
    {
        ROS_ERROR("Force Position Move Jiont mode wrong!\n");
    }
}

void GetSixForce_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = GetSixForce_Cmd();
    if (res == 0)
    {
        ROS_INFO("send GetSixForce cmd success!\n");
    }
    else
    {
        ROS_ERROR("send GetSixForce cmd failed!\n");
    }
}

void GetOneForce_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = GetOneForce_Cmd();
    if (res == 0)
    {
        ROS_INFO("send GetOneForce cmd success!\n");
    }
    else
    {
        ROS_ERROR("send GetOneForce cmd failed!\n");
    }
}

void ClearForceData_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("ClearForceData_Callback!\n");

    int res = 0;
    res = ClearForceData_Cmd();
    if (res == 0)
    {
        ROS_INFO("send ClearForceData cmd success!\n");
    }
    else
    {
        ROS_ERROR("send ClearForceData cmd failed!\n");
    }
}

void SetForceSensor_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("SetForceSensor_Callback!\n");

    int res = 0;
    res = SetForceSensor_Cmd();
    if (res == 0)
    {
        ROS_INFO("send SetForceSensor cmd success!\n");
    }
    else
    {
        ROS_ERROR("send SetForceSensor cmd failed!\n");
    }
}

void ManualSetForcePose_Callback(const rm_msgs::Manual_Set_Force_Pose msg)
{
    ROS_INFO("ManualSetForcePose_Callback!\n");

    int res = 0;
    long long joint[7];
    std::string pose;
    pose = msg.pose;
    for (int i = 0; i < 6; i++)
    {
        joint[i] = msg.joint[i];
    }
    if(arm_dof == 7)
    {
        joint[6] = msg.joint[6];
    }

    res = ManualSetForcePose_Cmd(pose,joint);
    if (res == 0)
    {
        ROS_INFO("send ManualSetForcePose cmd success!\n");
    }
    else
    {
        ROS_ERROR("send ManualSetForcePose cmd failed!\n");
    }
}

void StopSetForceSensor_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("StopSetForceSensor_Callback!\n");

    int res = 0;
    res = StopSetForceSensor_Cmd();
    if (res == 0)
    {
        ROS_INFO("send StopSetForceSensor cmd success!\n");
    }
    else
    {
        ROS_ERROR("send StopSetForceSensor cmd failed!\n");
    }
}

/***** ********************************END****************************************/

void JointPos_Callback(const rm_msgs::JointPos msg)
{
    int res = 0;
    int i = 0;
    float joint[7];
    float expand;
    uint8_t trajectory_mode;
    uint8_t radio;

    for (i = 0; i < 6; i++)
    {
        joint[i] = msg.joint[i] * RAD_DEGREE;
    }
    if(arm_dof == 7)
    {
        joint[6] = msg.joint[6] * RAD_DEGREE;
    }
    expand = msg.expand;
    trajectory_mode= trajectory_mode_;
    radio = radio_;
    res = Movej_CANFD(joint,expand, canfd_follow, trajectory_mode, radio);
    joint_flag = true;
    if (res == 0)
    {
        // ROS_INFO("JointPos success!\n");
    }
    else
    {
        ROS_ERROR("JointPos failed!\n");
    }
}

void MoveJ_Fd_Custom_Callback(const rm_msgs::JointPosCustom msg)
{
    int res = 0;
    int i = 0;
    float joint[7];
    float expand;
    bool follow;
    uint8_t trajectory_mode;
    uint8_t radio;

    for (i = 0; i < 6; i++)
    {
        joint[i] = msg.joint[i] * RAD_DEGREE;
    }
    if(arm_dof == 7)
    {
        joint[6] = msg.joint[6] * RAD_DEGREE;
    }
    expand = msg.expand;
    follow = msg.follow;
    trajectory_mode= msg.trajectory_mode;
    radio = msg.radio;
    res = Movej_CANFD(joint,expand, follow, trajectory_mode, radio);
    joint_flag = true;
    if (res == 0)
    {
        // ROS_INFO("JointPos success!\n");
    }
    else
    {
        ROS_ERROR("JointPos failed!\n");
    }
}

void Arm_DO_Callback(const rm_msgs::Arm_Digital_Output msg)
{
    int res = 0;
    if ((msg.num > 0) && (msg.num < 5))
    {
        res = Set_DO_State(msg.num, msg.state);
        if (res == 0)
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
    if ((msg.num > 0) && (msg.num < 5))
    {
        res = Set_AO_State(msg.num, msg.voltage);
        if (res == 0)
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
    if ((msg.num > 0) && (msg.num < 3))
    {
        res = Set_Tool_DO_State(msg.num, msg.state);
        if (res == 0)
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
    if (res == 0)
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

    if (msg.speed < 1)
    {
        msg.speed = 1;
    }
    else if (msg.speed > 1000)
    {
        msg.speed = 1000;
    }
    if (msg.force < 1)
    {
        msg.force = 1;
    }
    else if (msg.force > 1000)
    {
        msg.force = 1000;
    }

    res = Set_Gripper_Pick(msg.speed, msg.force);
    if (res == 0)
    {
        if (res == 0)
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

void Gripper_Pick_On_Callback(rm_msgs::Gripper_Pick msg)
{
    int res = 0;

    if (msg.speed < 1)
    {
        msg.speed = 1;
    }
    else if (msg.speed > 1000)
    {
        msg.speed = 1000;
    }
    if (msg.force < 1)
    {
        msg.force = 1;
    }
    else if (msg.force > 1000)
    {
        msg.force = 1000;
    }

    res = Set_Gripper_Pick_on(msg.speed, msg.force);
    if (res == 0)
    {
        if (res == 0)
        {
            RM_Joint.gripper_joint = 0;
            ROS_INFO("Gripper pick on success!\n");
        }
        else
        {
            ROS_ERROR("Gripper pick on failed!\n");
        }
    }
}

void Gripper_Set_Callback(rm_msgs::Gripper_Set msg)
{
    int res = 0;

    if (msg.position < 1)
    {
        msg.position = 1;
    }
    else if (msg.position > 1000)
    {
        msg.position = 1000;
    }

    res = Set_Gripper(msg.position);
    if (res == 0)
    {
        if (res == 0)
        {
            RM_Joint.gripper_joint = GRIPPER_WIDTH * msg.position / 2000;
            ROS_INFO("Gripper set success!\n");
        }
        else
        {
            ROS_ERROR("Gripper set failed!\n");
        }
    }
}
void Stop_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = Move_Stop_Cmd();
    if (res == 0)
    {
        ROS_INFO("Emergency stop success!\n");
    }
    else
    {
        ROS_ERROR("Emergency stop failed!\n");
    }
}

void Joint_Enable_Callback(const rm_msgs::Joint_Enable msg)
{
    int res = 0;
    if ((msg.joint_num > 7) || (msg.joint_num < 1))
    {
        ROS_ERROR("Joint Enable Set Error:Joint num out of range");
        return;
    }
    // Enable Joint, Firstly, clear joint err
    if (msg.state)
    {
        res = Clear_Joint_Err(msg.joint_num);
        if (res != 0)
        {
            ROS_ERROR("Joint Enable Set Failed");
            return;
        }
    }
    res = Set_Joint_Enable(msg.joint_num, msg.state);
    if (res != 0)
    {
        ROS_ERROR("Joint Enable Set Failed");
        return;
    }
}
void System_Enable_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("System_Enable_Callback!\n");

    int res = 0;
    res = clearsystemerr_Cmd();
    if (res == 0)
    {
        ROS_INFO("send System_Enable cmd success!\n");
    }
    else
    {
        ROS_ERROR("send System_Enable cmd failed!\n");
    }
}

void IO_Update_Callback(const rm_msgs::IO_Update msg)
{
    int res = 0;
    // Arm IO Update
    if (msg.type == 1)
    {
        res = Get_IO_Input();
    }
    // Tool IO Update
    else if (msg.type == 2)
    {
        res = Get_Tool_IO_Input();
    }
    if (res != 0)
    {
        ROS_ERROR("Joint Enable Set Failed");
        return;
    }
}

/***** ********************************START***************************************
 * 20210901修改: 增加对Turtle底盘的控制相关
 * 订阅控制Turtle的主题接收到数据后的处理函数
 * *********************************************************************************/
void TurtleCtr_Callback(const rm_msgs::Turtle_Driver msg)
{
    // ROS_INFO("TurtleCtr_Callback!\n");
    std::string message_type = msg.message_type;
    std::string robot_macAddr = msg.robot_mac_address;
    float vx = msg.vx;
    float vy = msg.vy;
    float vtheta = msg.vtheta;
    ROS_INFO("recv turtle control message[message_type:%s  robot_macAddr:%s  vx:%f  vy:%f  vtheta:%f\r\n", message_type.c_str(), robot_macAddr.c_str(), vx, vy, vtheta);

    int res = 0;
    res = SendTurtleCtrCmd(message_type, robot_macAddr, vx, vy, vtheta);
    if (res == 0)
    {
        ROS_INFO("Turtle control success!\n");
    }
    else
    {
        ROS_ERROR("Turtle control failed!\n");
    }
}

void getArmStateTimerSwitch_Callback(const std_msgs::Bool msg)
{
    if (msg.data)
    {
        if(CONTROLLER_VERSION == 1)
        State_Timer.stop();
        else
        {
            ;
            // set_realtime_push_flag = true;
            // Udp_State_Timer.stop();
        }
    }
    else
    {
        if(CONTROLLER_VERSION == 1)
        State_Timer.start();
        else
        {
            ;
            // set_realtime_push_flag = false;
            // Udp_State_Timer.start();
        }
    }
}


// Update:2023-7-25 @HermanYe
// Get controller version
void Get_Arm_Software_Version_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("Get_Arm_Software_Version_Callback!\n");
    int res = 0;
    res = Get_Arm_Software_Version();
    if(res != 0)
    {
        ROS_ERROR("Get_Arm_Software_Version cmd failed!\n");
    }
}
void Get_Joint_Software_Version_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("Get_Joint_Software_Version_Callback!\n");
    int res = 0;
    res = Get_Joint_Software_Version();
    if(res == 0){
        // ROS_INFO("Get_Joint_Software_Version_Callback success!\n");
    }
    else{
        ROS_ERROR("Get_Joint_Software_Version cmd failed!\n");
    }
}
void Get_Tool_Software_Version_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("Get_Tool_Software_Version_Callback!\n");
    int res = 0;
    res = Get_Tool_Software_Version();
    if(res == 0){
        // ROS_INFO("Get_Tool_Software_Version success!\n");
    }
    else{
        ROS_ERROR("Get_Tool_Software_Version cmd failed!\n");
    }
}



// ----------------------------------------------------------------------------
// Update:2025-05-13 @Poppy
// 四代控制器新增
// void Get_Arm_Software_Version_Callback_v4(const std_msgs::Empty msg)
// {
//     ROS_INFO("Get_Arm_Software_Version_Callback!\n");
//     int res = 0;
//     res = Get_Arm_Software_Version();
//     if(res != 0)
//     {
//         ROS_ERROR("Get_Arm_Software_Version_Callback cmd failed!\n");
//     }
// }

// 设置机械臂急停状态
void Set_Arm_Emergency_Stop_Callback(const std_msgs::Bool msg)
{
    ROS_INFO("Set_Arm_Emergency_Stop_Callback!\n");
    int res = 0;
    res = Parse_Set_Arm_Emergency_Stop(msg.data);
    if(res == 0){
        // ROS_INFO("Set_Arm_Emergency_Stop_Callback success!\n");
    }
    else{
        ROS_ERROR("Set_Arm_Emergency_Stop_Callback cmd failed!\n");
    }
}

void Get_Trajectory_File_List_Callback(const rm_msgs::Gettrajectorylist msg)
{
    ROS_INFO("Get_Trajectory_File_List_Callback!\n");
    int res = 0;
    res = Get_Trajectory_File_List(msg.page_num,msg.page_size,msg.vague_search.data());
    if(res == 0){
        // ROS_INFO("Get_Trajectory_File_List_Callback success!\n");
    }
    else{
        ROS_ERROR("Get_Trajectory_File_List_Callback cmd failed!\n");
    }
}
void Set_Run_Trajectory_Callback(const std_msgs::String msg)
{
    ROS_INFO("Set_Run_Trajectory_Callback!\n");
    int res = 0;
    res = Set_Run_Trajectory(msg.data);
    if(res == 0){
        // ROS_INFO("Set_Run_Trajectory success!\n");
    }
    else{
        ROS_ERROR("Set_Run_Trajectory cmd failed!\n");
    }
}
void Delete_Trajectory_File_Callback(const std_msgs::String msg)
{
    ROS_INFO("Delete_Trajectory_File_Callback!\n");
    int res = 0;
    res = Delete_Trajectory_File(msg.data);
    if(res == 0){
        // ROS_INFO("Delete_Trajectory_File success!\n");
    }
    else{
        ROS_ERROR("Delete_Trajectory_File cmd failed!\n");
    }
}
void Save_Trajectory_File_Callback(const std_msgs::String msg)
{
    ROS_INFO("Save_Trajectory_File_Callback!\n");
    int res = 0;
    res = Save_Trajectory_File(msg.data);
    if(res == 0){
        // ROS_INFO("Save_Trajectory_File success!\n");
    }
    else{
        ROS_ERROR("Save_Trajectory_File cmd failed!\n");
    }
}

void Get_Flowchart_Program_Run_State_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("Get_Flowchart_Program_Run_State_Callback!\n");
    int res = 0;
    res = Get_Flowchart_Program_Run_State();
    if(res == 0){
        // ROS_INFO("Get_Flowchart_Program_Run_State success!\n");
    }
    else{
        ROS_ERROR("Get_Flowchart_Program_Run_State cmd failed!\n");
    }
}

void Movel_Offset_Callback(const rm_msgs::Moveloffset msg)
{
    ROS_INFO("Movel_Offset_Callback!\n");
    int res = 0;
    res = Movel_Offset(msg.offset, msg.speed, msg.r, msg.trajectory_connect, msg.frame_type, msg.block);
    if(res == 0){
        // ROS_INFO("Movel_Offset success!\n");
    }
    else{
        ROS_ERROR("Movel_Offset cmd failed!\n");
    }
}



// ----------------------------------ModbusTCP主站相关-------------------------------------
void Add_Modbus_Tcp_Master_Callback(const rm_msgs::Modbustcpmasterinfo msg)
{
    ROS_INFO("Add_Modbus_Tcp_Master_Callback!\n");
    int res = 0;
    res = Add_Modbus_Tcp_Master(msg.master_name,msg.ip,msg.port);
    if(res == 0){
        // ROS_INFO("Movel_Offset success!\n");
    }
    else{
        ROS_ERROR("Add_Modbus_Tcp_Master_Callback cmd failed!\n");
    }
}
void Update_Modbus_Tcp_Master_Callback(const rm_msgs::UpdateTCPmasterparam msg)
{
    ROS_INFO("Update_Modbus_Tcp_Master_Callback!\n");
    int res = 0;
    res = Update_Modbus_Tcp_Master(msg.master_name,msg.new_name,msg.ip,msg.port);
    if(res == 0){
        // ROS_INFO("Movel_Offset success!\n");
    }
    else{
        ROS_ERROR("Update_Modbus_Tcp_Master_Callback cmd failed!\n");
    }
}
void Delete_Modbus_Tcp_Master_Callback(const std_msgs::String msg)
{
    ROS_INFO("Delete_Modbus_Tcp_Master_Callback!\n");
    int res = 0;
    res = Delete_Modbus_Tcp_Master(msg.data);
    if(res == 0){
        // ROS_INFO("Movel_Offset success!\n");
    }
    else{
        ROS_ERROR("Delete_Modbus_Tcp_Master_Callback cmd failed!\n");
    }
}
void Get_Modbus_Tcp_Master_Callback(const std_msgs::String msg)
{
    ROS_INFO("Get_Modbus_Tcp_Master_Callback!\n");
    int res = 0;
    res = Get_Modbus_Tcp_Master(msg.data);
    if(res == 0){
        // ROS_INFO("Movel_Offset success!\n");
    }
    else{
        ROS_ERROR("Get_Modbus_Tcp_Master_Callback cmd failed!\n");
    }
}
void Get_Modbus_Tcp_Master_List_Callback(const rm_msgs::Get_TCP_Master_List_Param msg)
{
    ROS_INFO("Get_Modbus_Tcp_Master_List_Callback!\n");
    int res = 0;
    res = Get_Modbus_Tcp_Master_List(msg.page_num,msg.page_size,msg.vague_search);
    if(res == 0){
        // ROS_INFO("Movel_Offset success!\n");
    }
    else{
        ROS_ERROR("Get_Modbus_Tcp_Master_List_Callback cmd failed!\n");
    }
}


void Set_Controller_Rs485_Mode_Callback(const rm_msgs::RS485params msg)
{
    ROS_INFO("Set_Controller_Rs485_Mode_Callback!\n");
    int res = 0;
    res = Set_Controller_Rs485_Mode(msg.mode,msg.baudrate);
    if(res == 0){
        // ROS_INFO("Movel_Offset success!\n");
    }
    else{
        ROS_ERROR("Set_Controller_Rs485_Mode_Callback cmd failed!\n");
    }
}

void Get_Controller_Rs485_Mode_V4_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("Get_Controller_Rs485_Mode_V4_Callback!\n");
    int res = 0;
    res = Get_Controller_Rs485_Mode_V4();
    if(res == 0){
        // ROS_INFO("Movel_Offset success!\n");
    }
    else{
        ROS_ERROR("Get_Controller_Rs485_Mode_V4_Callback cmd failed!\n");
    }
}
void Set_Tool_Rs485_Mode_Callback(const rm_msgs::RS485params msg)
{
    ROS_INFO("Set_Tool_Rs485_Mode_Callback!\n");
    int res = 0;
    res = Set_Tool_Rs485_Mode(msg.mode,msg.baudrate);
    if(res == 0){
        ROS_INFO("Set_Tool_Rs485_Mode_Callback success!\n");
    }
    else{
        ROS_ERROR("Set_Tool_Rs485_Mode_Callback cmd failed!\n");
    }
}
void Get_Tool_Rs485_Mode_V4_Callback(const std_msgs::Empty msg)
{
    ROS_INFO("Get_Tool_Rs485_Mode_V4_Callback!\n");
    int res = 0;
    res = Get_Tool_Rs485_Mode_V4();
    if(res == 0){
        // ROS_INFO("Get_Tool_Rs485_Mode_V4_Callback success!\n");
    }
    else{
        ROS_ERROR("Get_Tool_Rs485_Mode_V4_Callback cmd failed!\n");
    }
}

//**********************************2025.6.17************************************** */
void Read_Modbus_Coils_Callback(const rm_msgs::Read_TCPandRTU msg)
{
    if(msg.type == 3)
    {   
        ROS_INFO("Read_Modbus_Tcp_Coils_Callback!\n");
        int res = 0;
        res = Read_Modbus_Tcp_Coils(msg.address,msg.num,msg.ip,msg.port,msg.master_name);
        if(res == 0){
            //ROS_INFO("Read_Modbus_Tcp_Coils_Port_Callback success!\n");
        }
        else{
            ROS_ERROR("Read_Modbus_Tcp_Coils_Callback cmd failed!\n");
        }
    }
    else if(msg.type == 0 || msg.type == 1)
    {
        ROS_INFO("Read_Modbus_Rtu_Coils_Callback!\n");
        int res = 0;
        res = Read_Modbus_Rtu_Coils(msg.address,msg.device,msg.num,msg.type);
        if(res == 0){
            // ROS_INFO("Read_Modbus_Rtu_Coils_Callback success!\n");
        }
        else{
            ROS_ERROR("Read_Modbus_Rtu_Coils_Callback cmd failed!\n");
        }
    }
    else
    {
        ROS_ERROR("Invalid Read_TCPandRTU message format! Missing required fields.\n");
    }
}

void Write_Modbus_Coils_Callback(const rm_msgs::Write_TCPandRTU msg)
{
    if(msg.type == 3)
    {
        ROS_INFO("Write_Modbus_Tcp_Coils_Callback!\n");
        int res = 0;
        res = Write_Modbus_Tcp_Coils(msg.address,msg.data,msg.ip,msg.port,msg.master_name);
        if(res == 0){
            // ROS_INFO("Write_Modbus_Tcp_Coils_Callback success!\n");
        }
        else{
            ROS_ERROR("Write_Modbus_Tcp_Coils_Callback cmd failed!\n");
        }
    }
    else if(msg.type == 0 || msg.type == 1)
    {
        ROS_INFO("Write_Modbus_Rtu_Coils_Callback!\n");
        int res = 0;
        res = Write_Modbus_Rtu_Coils(msg.address,msg.data,msg.type,msg.device);
        if(res == 0){
            // ROS_INFO("Write_Modbus_Rtu_Coils_Callback success!\n");
        }
        else{
            ROS_ERROR("Write_Modbus_Rtu_Coils_Callback cmd failed!\n");
        }
    }
    else
    {
        ROS_ERROR("Invalid Write_TCPandRTU message format! Missing required fields.\n");
    }

}

void Read_Modbus_Input_Status_Callback(const rm_msgs::Read_TCPandRTU msg)
{
    if(msg.type == 3)
    {
        ROS_INFO("Read_Modbus_Tcp_Input_Status_Callback!\n");
        int res = 0;
        res = Read_Modbus_Tcp_Input_Status(msg.address,msg.num,msg.ip,msg.port,msg.master_name);
        if(res == 0){
            // ROS_INFO("Read_Modbus_Tcp_Input_Status_Callback success!\n");
        }
        else{
            ROS_ERROR("Read_Modbus_Tcp_Input_Status_Callback cmd failed!\n");
        }       
    }
    else if(msg.type == 0 || msg.type == 1)
    {
        ROS_INFO("Read_Modbus_Rtu_Input_Status_Callback!\n");
        int res = 0;
        res = Read_Modbus_Rtu_Input_Status(msg.address,msg.device,msg.num,msg.type);
        if(res == 0){
            // ROS_INFO("Read_Modbus_Rtu_Input_Status_Callback success!\n");
        }
        else{
            ROS_ERROR("Read_Modbus_Rtu_Input_Status_Callback cmd failed!\n");
        }
    }
    else
    {
        ROS_ERROR("Invalid Read_TCPandRTU message format! Missing required fields.\n");
    }

}

void Read_Modbus_Holding_Registers_Callback(const rm_msgs::Read_TCPandRTU msg)
{
    if(msg.type == 3)
    {
        ROS_INFO("Read_Modbus_TCP_Holding_Registers_Callback!\n");
        int res = 0;
        res = Read_Modbus_TCP_Holding_Registers(msg.address,msg.num,msg.ip,msg.port,msg.master_name);
        if(res == 0){
            // ROS_INFO("Read_Modbus_TCP_Holding_Registers_Callback success!\n");
        }
        else{
            ROS_ERROR("Read_Modbus_TCP_Holding_Registers_Callback cmd failed!\n");
        }
    }
    else if(msg.type == 0 || msg.type == 1)
    {
        ROS_INFO("Read_Modbus_Rtu_Holding_Registers_Callback!\n");
        int res = 0;
        res = Read_Modbus_Rtu_Holding_Registers(msg.address,msg.device,msg.num,msg.type);
        if(res == 0){
            // ROS_INFO("Read_Modbus_Rtu_Holding_Registers_Callback success!\n");
        }
        else{
            ROS_ERROR("Read_Modbus_Rtu_Holding_Registers_Callback cmd failed!\n");
        }
    }
    else
    {
        ROS_ERROR("Invalid Read_TCPandRTU message format! Missing required fields.\n");
    }
}

void Write_Modbus_Registers_Callback(const rm_msgs::Write_TCPandRTU msg)
{
    if(msg.type == 3)
    {
        ROS_INFO("Write_Modbus_Tcp_Registers_Callback!\n");
        int res = 0;
        res = Write_Modbus_Tcp_Registers(msg.address,msg.data,msg.ip,msg.port,msg.master_name);
        if(res == 0){
            // ROS_INFO("Write_Modbus_Tcp_Registers_Callback success!\n");
        }
        else{
            ROS_ERROR("Write_Modbus_Tcp_Registers_Callback cmd failed!\n");
        } 
    }
    else if(msg.type == 0 || msg.type == 1)
    {
        ROS_INFO("Write_Modbus_Rtu_Registers_Callback!\n");
        int res = 0;
        res = Write_Modbus_Rtu_Registers(msg.address,msg.data,msg.type,msg.device);
        if(res == 0){
            // ROS_INFO("Write_Modbus_Rtu_Registers_Callback success!\n");
        }
        else{
            ROS_ERROR("Write_Modbus_Rtu_Registers_Callback cmd failed!\n");
        }
    }
    else
    {
        ROS_ERROR("Invalid Write_TCPandRTU message format! Missing required fields.\n");
    }
}

void Read_Modbus_Input_Registers_Callback(const rm_msgs::Read_TCPandRTU msg)
{
    if(msg.type == 3)
    {
        ROS_INFO("Read_Modbus_Tcp_Input_Registers_Callback!\n");
        int res = 0;
        res = Read_Modbus_Tcp_Input_Registers(msg.address,msg.num,msg.ip,msg.port,msg.master_name);
        if(res == 0){
            // ROS_INFO("Read_Modbus_Tcp_Input_Registers_Callback success!\n");
        }
        else{
            ROS_ERROR("Read_Modbus_Tcp_Input_Registers_Callback cmd failed!\n");
        }
    }
    else if(msg.type == 0 || msg.type == 1)
    {
        ROS_INFO("Read_Modbus_Rtu_Input_Registers_Callback!\n");
        int res = 0;
        res = Read_Modbus_Rtu_Input_Registers(msg.address,msg.device,msg.num,msg.type);
        if(res == 0){
            // ROS_INFO("Read_Modbus_Rtu_Input_Registers_Callback success!\n");
        }
        else{
            ROS_ERROR("Read_Modbus_Rtu_Input_Registers_Callback cmd failed!\n");
        }
    }
    else
    {
        ROS_ERROR("Invalid Read_TCPandRTU message format! Missing required fields.\n");
    }

}

// -------------------------------适配四代控制器end-------------------------------------
void Movep_Fd_Callback(const rm_msgs::CartePos msg)
{
    // ROS_INFO("enter Movep_Fd_Callback");
    int res = 0;
    byte speed;
    POSE target;
    bool follow;
    uint8_t trajectory_mode;
    uint8_t radio;
    target = Quater_To_Euler(msg.Pose);
    follow = canfd_follow;
    trajectory_mode= trajectory_mode_;
    radio = radio_;
    res = Movep_CANFD(target, follow, trajectory_mode, radio);
    joint_flag = true;
    if (res == 0)
    {
        // ROS_INFO("Movep_Fd success!\n");
    }
    else
    {
        ROS_ERROR("Movep_Fd failed!\n");
    }
}
// ----------------------------------------------------------------------------
void Movep_Fd_Custom_Callback(const rm_msgs::CartePosCustom msg)
{
    // ROS_INFO("enter Movep_Fd_Callback");
    int res = 0;
    byte speed;
    POSE target;
    bool follow;
    uint8_t trajectory_mode;
    uint8_t radio;
    target = Quater_To_Euler(msg.Pose);
    follow = msg.follow;
    trajectory_mode= msg.trajectory_mode;
    radio = msg.radio;
    res = Movep_CANFD(target, follow, trajectory_mode, radio);
    joint_flag = true;
    if (res == 0)
    {
        // ROS_INFO("Movep_Fd success!\n");
    }
    else
    {
        ROS_ERROR("Movep_Fd failed!\n");
    }
}

/*****************************设置Udp参数配置***************************/
void Set_Realtime_Push_callback(const rm_msgs::Set_Realtime_Push msg)
{
    int res = 0;
    uint16_t cycle;
    bool enable;
    uint16_t port;
    uint16_t force_coordinate;
    std::string ip;
    cycle = msg.cycle;
    // enable = msg.enable;
    port = msg.port;
    ip = msg.ip;
    force_coordinate = msg.force_coordinate;
    Udp_Setting.custom_set_data.aloha_state_ = msg.aloha_state_enable;
    Udp_Setting.custom_set_data.arm_current_status_ = msg.arm_current_status_enable;
    Udp_Setting.custom_set_data.expand_state_ = msg.expand_state_enable;
    Udp_Setting.custom_set_data.hand_ = msg.hand_enable;
    Udp_Setting.custom_set_data.joint_acc_ = msg.joint_acc_enable;
    Udp_Setting.custom_set_data.joint_speed_ = msg.joint_speed_enable;
    Udp_Setting.custom_set_data.lift_state_ = msg.lift_state_enable;
    Udp_Setting.custom_set_data.tail_end_ = msg.tail_end_enable;
    Udp_Setting.custom_set_data.rm_plus_state_ = msg.rm_plus_state_enable;
    Udp_Setting.custom_set_data.rm_plus_base_ = msg.rm_plus_base_enable;
    res = Udp_Set_Realtime_Push(cycle, port, force_coordinate, ip, Udp_Setting.custom_set_data);
    if(res == 0)
    {
        ROS_INFO("Set_Realtime success!\n");
    }
    else
    {
        ROS_ERROR("Set_Realtime failed!\n");
    }
}
/****************************************************************************************/

/*****************************查询当前的Udp的参数配置*****************************************/
void Get_Realtime_Push_callback(const std_msgs::Empty msg)
{
    // ROS_INFO("Get_Realtime_Push_callback!\n");

    int res = 0;
    res = GetRealtimePush_Cmd();
    if (res == 0)
    {
        ROS_INFO("Get realtime_push cmd success!\n");
    }
    else
    {
        ROS_ERROR("Get realtime_push cmd failed!\n");
    }
}

/**************************************END****************************************/

/***************************Modbus Set function （三代控制器）***********************************/
void Set_RS485_Callback(const std_msgs::UInt32 msg)
{
    u_int32_t baudrate_;
    baudrate_ = msg.data;
    int res = 0;
    res = Set_RS485_Cmd(baudrate_);
    if(res==0)
    {
        ROS_INFO("Set RS485 cmd success!\n");
    }
    else
    {
        ROS_INFO("Set RS485 cmd failed!\n");
    }
}

void Get_Controller_RS485_Mode_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = Get_Controller_RS485_Mode_Cmd();
    if(res==0)
    {
        ROS_INFO("Get Controller RS485 Mode cmd success!\n");
    }
    else
    {
        ROS_INFO("Get Controller RS485 Mode cmd failed!\n");
    }
}

void Get_Tool_RS485_Mode_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = Get_Tool_RS485_Mode_Cmd();
    if(res==0)
    {
        ROS_INFO("Get Tool RS485 Mode cmd success!\n");
    }
    else
    {
        ROS_INFO("Get Tool RS485 Mode cmd failed!\n");
    }
}

void Set_Modbus_Mode_Callback(const rm_msgs::Set_Modbus_Mode msg)
{
    int res = 0;
    res = Set_Modbus_Mode_Cmd(msg.port, msg.baudrate, msg.timeout);
    if(res==0)
    {
        ROS_INFO("Set Modbus Mode cmd success!\n");
    }
    else
    {
        ROS_INFO("Set Modbus Mode cmd failed!\n");
    }
}

void Close_Modbus_Mode_Callback(const std_msgs::UInt8 msg)
{
    int res = 0;
    res = Close_Modbus_Mode_Cmd(msg.data);
    if(res==0)
    {
        ROS_INFO("Close Modbus Mode cmd success!\n");
    }
    else
    {
        ROS_INFO("Close Modbus Mode cmd failed!\n");
    }
}

void Set_Modbustcp_Mode_Callback(const rm_msgs::Set_Modbus_Mode msg)
{
    int res = 0;
    res = Set_Modbustcp_Mode_Cmd(msg.ip, msg.port, msg.timeout);
    if(res==0)
    {
        ROS_INFO("Set Modbustcp Mode cmd success!\n");
    }
    else
    {
        ROS_INFO("Set Modbustcp Mode cmd failed!\n");
    }
}

void Close_Modbustcp_Mode_Callback(const std_msgs::Empty msg)
{
    int res = 0;
    res = Close_Modbustcp_Mode_Cmd();
    if(res==0)
    {
        ROS_INFO("Close Modbustcp Mode cmd success!\n");
    }
    else
    {
        ROS_INFO("Close Modbustcp Mode cmd failed!\n");
    }
}

/***************************Modbus Infomation Read/Write function（三代控制器）******************/
void Read_Coils_Callback(const rm_msgs::Read_Register msg)
{
    modbus_data.read_coils.data.resize(1);
    int res=0;
    res = Read_Coils_Cmd(msg.port, msg.address, msg.num, msg.device);
    if(res==0)
    {
        ROS_INFO("Read Coils cmd success!\n");
    }
    else
    {
        ROS_INFO("Read Coils cmd failed!\n");
    }
}
void Read_Multiple_Coils_Callback(const rm_msgs::Read_Register msg)
{
    modbus_data.read_multiple_coils.data.resize(msg.num);
    int res=0;
    res = Read_Multiple_Coils_Cmd(msg.port, msg.address, msg.num, msg.device);
    if(res==0)
    {
        ROS_INFO("Read Multiple Coils cmd success!\n");
    }
    else
    {
        ROS_INFO("Read Multiple Coils cmd failed!\n");
    }
}
void Write_Single_Coil_Callback(const rm_msgs::Write_Register msg)
{
    int res=0;  
    if(msg.data.size() == 1)
    {
        uint16_t data;
        data = msg.data[0];
        res = Write_Single_Coil_Cmd(msg.port, msg.address, data, msg.device);
        if(res==0)
        {
            ROS_INFO("Write Single Coil cmd success!\n");
        }
        else
        {
            ROS_INFO("Write Single Coil cmd failed!\n");
        }
    }
    else
        ROS_INFO("Write Single Coil cmd data more than one!\n");
}
void Write_Coils_Callback(const rm_msgs::Write_Register msg)
{
    int res=0;
    // if(msg.num == msg.data.size())
    // {   
        std::vector<uint16_t> data;
        data.resize(msg.data.size());
        for(int i = 0;i<msg.data.size();i++)
        {
            data[i] = msg.data[i];
        }
        res = Write_Coils_Cmd(msg.port, msg.address, msg.num, data, msg.device);
        if(res==0)
        {
            ROS_INFO("Write Coils cmd success!\n");
        }
        else
        {
            ROS_INFO("Write Coils cmd failed!\n");
        }
    // }
    // else
    //     ROS_INFO("Write Coils cmd num or data size is error!\n");
}
void Read_Input_Status_Callback(const rm_msgs::Read_Register msg)
{
    modbus_data.read_input_status.data.resize(1);
    int res=0;
    res = Read_Input_Status_Cmd(msg.port, msg.address, msg.num, msg.device);
    if(res==0)
    {
        ROS_INFO("Read Input Status cmd success!\n");
    }
    else
    {
        ROS_INFO("Read Input Status cmd failed!\n");
    }
}
void Read_Holding_Registers_Callback(const rm_msgs::Read_Register msg)
{
    modbus_data.read_holding_registers.data.resize(1);
    int res=0;
    res = Read_Holding_Registers_Cmd(msg.port, msg.address, msg.device);
    if(res==0)
    {
        ROS_INFO("Read Holding Registers cmd success!\n");
    }
    else
    {
        ROS_INFO("Read Holding Registers cmd failed!\n");
    }
}
void Write_Single_Register_Callback(const rm_msgs::Write_Register msg)
{
    int res=0;
    if(msg.data.size() == 1)
    {
        uint16_t data;
        data = msg.data[0];
        res = Write_Single_Register_Cmd(msg.port, msg.address, data, msg.device);
        if(res==0)
        {
            ROS_INFO("Write Single Register cmd success!\n");
        }
        else
        {
            ROS_INFO("Write Single Register cmd failed!\n");
        }
    }
    else
        ROS_INFO("Write Single Register cmd data more than one!\n");
}
void Write_Registers_Callback(const rm_msgs::Write_Register msg)
{
    int res=0;
    if(msg.num == (msg.data.size()/2))
    {   
        std::vector<uint16_t> data;
        data.resize(msg.data.size());
        for(int i = 0;i<msg.data.size();i++)
        {
            data[i] = msg.data[i];
        }
        res = Write_Registers_Cmd(msg.port, msg.address, msg.num, data, msg.device);
        if(res==0)
        {
            ROS_INFO("Write Registers cmd success!\n");
        }
        else
        {
            ROS_INFO("Write Registers cmd failed!\n");
        }
    }
    else
        ROS_ERROR("Write Registers cmd num or data size is error!\n");
}
void Read_Multiple_Holding_Registers_Callback(const rm_msgs::Read_Register msg)
{
    modbus_data.read_multiple_holding_registers.data.resize(msg.num);
    int res=0;
    res = Read_Multiple_Holding_Registers_Cmd(msg.port, msg.address, msg.num, msg.device);
    if(res==0)
    {
        ROS_INFO("Read Multiple Holding Registers cmd success!\n");
    }
    else
    {
        ROS_INFO("Read Multiple Holding Registers cmd failed!\n");
    }
}
void Read_Input_Registers_Callback(const rm_msgs::Read_Register msg)
{
    modbus_data.read_input_registers.data.resize(1);
    int res=0;
    res = Read_Input_Registers_Cmd(msg.port, msg.address, msg.device);
    if(res==0)
    {
        ROS_INFO("Read Input Registers cmd success!\n");
    }
    else
    {
        ROS_INFO("Read Input Registers cmd failed!\n");
    }
}
void Read_Multiple_Input_Registers_Callback(const rm_msgs::Read_Register msg)
{
    modbus_data.read_multiple_input_registers.data.resize(msg.num);
    int res=0;
    res = Read_Multiple_Input_Registers_Cmd(msg.port, msg.address, msg.num, msg.device);
    if(res==0)
    {
        ROS_INFO("Read Multiple Input Registers cmd success!\n");
    }
    else
    {
        ROS_INFO("Read Multiple Input Registers cmd failed!\n");
    }
}

void SetRmPlusMode_Callback(const std_msgs::UInt32 msg)
{
    int res=0;
    res = Set_Rm_Plus_Mode_Cmd(msg.data);
    if(res==0)
    {
        ROS_INFO("Set_Rm_Plus_Mode cmd success!\n");
    }
    else
    {
        ROS_INFO("Set_Rm_Plus_Mode cmd failed!\n");
    }
}
void GetRmPlusMode_Callback(const std_msgs::Empty msg)
{
    int res=0;
    res = Get_Rm_Plus_Mode_Cmd();
    if(res==0)
    {
        ROS_INFO("Get_Rm_Plus_Mode_Cmd success!\n");
    }
    else
    {
        ROS_INFO("Get_Rm_Plus_Mode_Cmd failed!\n");
    }
}
void SetRmPlusTouch_Callback(const std_msgs::UInt16 msg)
{
    int res=0;
    res = Set_Rm_Plus_Touch_Cmd(msg.data);
    if(res==0)
    {
        ROS_INFO("Set_Rm_Plus_Touch cmd success!\n");
    }
    else
    {
        ROS_INFO("Set_Rm_Plus_Touch cmd failed!\n");
    }
}

void GetRmPlusTouch_Callback(const std_msgs::Empty msg)
{
    int res=0;
    res = Get_Rm_Plus_Touch_Cmd();
    if(res==0)
    {
        ROS_INFO("Get_Rm_Plus_Touch_Cmd success!\n");
    }
    else
    {
        ROS_INFO("Get_Rm_Plus_Touch_Cmd failed!\n");
    }
}
/**********************************V2.6.0************************************/
void SetArmPause_Callback(const std_msgs::Empty msg)
{
    int res=0;
    res = Set_Rm_Arm_Pause_Cmd();
    if(res==0)
    {
        ROS_INFO("Set_Rm_Arm_Continue_Cmd success!\n");
    }
    else
    {
        ROS_INFO("Set_Rm_Arm_Continue_Cmd failed!\n");
    }
}
void SetArmContinue_Callback(const std_msgs::Empty msg)
{
    int res=0;
    res = Set_Rm_Arm_Continue_Cmd();
    if(res==0)
    {
        ROS_INFO("Set_Rm_Arm_Continue_Cmd success!\n");
    }
    else
    {
        ROS_INFO("Set_Rm_Arm_Continue_Cmd failed!\n");
    }
}
void GetExpandState_Callback(const std_msgs::Empty msg)
{
    int res=0;
    res = Get_Expand_State_Cmd();
    if(res==0)
    {
        ROS_INFO("Get_Expand_State_Cmd success!\n");
    }
    else
    {
        ROS_INFO("Get_Expand_State_Cmd failed!\n");
    }
}
void SetExpandSpeed_Callback(const rm_msgs::Expand_Speed msg)
{
    int res=0;
    res = Set_Expand_Speed_Cmd(msg.speed);
    if(res==0)
    {
        ROS_INFO("Set_Expand_Speed_Cmd success!\n");
    }
    else
    {
        ROS_INFO("Set_Expand_Speed_Cmd failed!\n");
    }
}
void SetExpandPos_Callback(const rm_msgs::Expand_Position msg)
{
    int res=0;
    res = Set_Expand_Pos_Cmd(msg.pos, msg.speed);
    if(res==0)
    {
        ROS_INFO("Set_Expand_Pos_Cmd success!\n");
    }
    else
    {
        ROS_INFO("Set_Expand_Pos_Cmd failed!\n");
    }
}
void timer_callback(const ros::TimerEvent)
{

    if (timer_cnt < 100)
    {
        Get_Arm_Joint();
        timer_cnt++;
    }
    else
    {
        Get_Joint_Err_Flag();
        timer_cnt = 0;
    }
}




struct sockaddr_in clientAddr;
socklen_t clientAddrLen = sizeof(clientAddr);
char udp_socket_buffer[1800];

bool read_data()
{
    memset(udp_socket_buffer, 0, sizeof(udp_socket_buffer));

    ssize_t numBytes = recvfrom(Udp_Sockfd, udp_socket_buffer, sizeof(udp_socket_buffer), 0,
        (struct sockaddr*) & clientAddr, &clientAddrLen);
    if ((numBytes < 0)||(tcp_arm_joint_state == false)) {
        // std::cerr << "Error in recvfrom" << std::endl;
        // std::cerr << "numBytes:" << numBytes << std::endl;
        // std::cerr << "tcp_arm_joint_state:" << tcp_arm_joint_state << std::endl;
        close(Udp_Sockfd);
        return false;
    }
    // 将接收到的数据输出到控制台
    udp_socket_buffer[numBytes] = '\0'; // 添加字符串结束符
            // std::cout << "Received from "                                         //<< inet_ntoa(clientAddr.sin_addr)
            // << ":" << ntohs(clientAddr.sin_port) << ": "
            // << udp_socket_buffer << std::endl;
    if((udp_socket_buffer[numBytes-2]==0X0D)&&(udp_socket_buffer[numBytes-1]==0X0A))
    {
        return true;
    }
    else
    {
        // ROS_ERROR("udp_socket_buffer IS error");
        return false;
    }
}

/*UDP不断发布机械臂当前状态*/
void udp_timer_callback(const ros::TimerEvent)
{
    if(read_data()==true)
    {
        if(Parser_Udp_Msg(udp_socket_buffer)==0);
    }
    else
    {
        udp_failed_time++;
        if(((udp_failed_time >= 10)&&(realtime_arm_joint_state == true))||(tcp_arm_joint_state == false))
        {
            realtime_arm_joint_state = false;
            ROS_ERROR("Read_UDP_date IS error");
        }
    }
}

/*心跳包不断查看连通状态*/
void heart_callback(const ros::TimerEvent)
{
    /*23.8.4添加SocketConnected()
    *函数作用：判断连接状态
    *参数：   无
    *返回值   1：连接正常  0:连接断开
    */

    if(SocketConnected() == 1)
    {
        //ROS_INFO("Connect IS OK");
        connect_status = 1;
    }
    else
    {
        // ROS_ERROR("Read_UDP_date IS error111");
        connect_status = 0;
        close(Arm_Socket);
        if(CONTROLLER_VERSION == 1)
        {
            State_Timer.stop();
        }
        else
        {
            // ROS_ERROR("Read_UDP_date IS error222");
            Udp_State_Timer.stop();
            Udp_Socket_Close();
        }
        while (Arm_Socket_Start())
        {
            /*
            *作用当ctrl+c执行时，会调用下面的语句，进行退出相关操作
            */
            if(ctrl_flag)
            { 
                // my action when signal set it 1
                //标志位用以调用主函数中的ctrl+c的相关退出程序。
                ros_shutdown_flage=1; 
                break;
            }
            else
            {
                usleep(1000);
                // sleep(1);
            }
        }
        // 在这里将connect_status更换为1，若未建立连接，程序是不会走到这里的，会一直卡死在上面的while循环里。
        if(CONTROLLER_VERSION == 1)
        {
        State_Timer.start();
        }
        else
        {
            while (Udp_Socket_Start())
            {
                /*
                *作用当ctrl+c执行时，会调用下面的语句，进行退出相关操作
                */
                if(ctrl_flag)
                { 
                    // my action when signal set it 1
                    //标志位用以调用主函数中的ctrl+c的相关退出程序。
                    ros_shutdown_flage=1; 
                    break;
                }
                else
                {
                    sleep(1);
                }
            }  
            udp_failed_time = 0;
            realtime_arm_joint_state = true;
            Udp_State_Timer.start();
        }
        connect_status = 1;
        tcp_arm_joint_state = true;
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_("~");
    ros::Rate loop_rate(500); // 200Hz,5ms
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

    char socket_buffer[500];
    memset(socket_buffer, 0, sizeof(socket_buffer));
    int buffer_cnt = 0;

    // msg
    rm_msgs::Arm_IO_State Arm_IO;
    rm_msgs::Tool_IO_State Tool_IO;
    rm_msgs::Plan_State Plan;
    rm_msgs::ChangeTool_State ChangeTool_State;
    rm_msgs::ChangeWorkFrame_State ChangeWorkFrame_State;
    rm_msgs::Arm_Current_State Arm_State;
    rm_msgs::Force_Position_State Force_Position_State;
    rm_msgs::Six_Force Six_Force;
    rm_msgs::Six_Force zero_force;
    rm_msgs::Six_Force work_zero_force;
    rm_msgs::Six_Force tool_zero_force;
    std_msgs::Bool state;
    std_msgs::UInt32 plus_mode_result;
    std_msgs::UInt16 plus_touch_result;
    rm_msgs::Joint_Current joint_current;
    rm_msgs::ArmState armState;
    rm_msgs::LiftState liftState;
    rm_msgs::ExpandState expandState;

    /***********************************UDP控制数据类型*************************************/ 
    rm_msgs::Set_Realtime_Push udp_set_realtime_push;

    geometry_msgs::Quaternion quaternion_msg;
    tf2::Quaternion quaternion_tf;

    private_nh_.param<std::string>("Arm_IP",               Arm_IP_,                "192.168.1.18");
    private_nh_.param<std::string>("Arm_Type",             RM_Joint.product_version,      "RM75");
    private_nh_.param<bool>       ("Follow",               canfd_follow,           false);
    private_nh_.param<int>        ("Arm_Port",             Arm_Port_,              8080);
    private_nh_.param<int>        ("Arm_Dof",              arm_dof,                6);
    private_nh_.param<std::string>("Udp_IP",               Udp_IP_,                "192.168.1.10");
    private_nh_.param<int>        ("Udp_Port",             Udp_Port_,              8089);
    private_nh_.param<int>        ("Udp_cycle",            Udp_cycle_,             5);
    private_nh_.param<int>        ("Udp_force_coordinate", Udp_force_coordinate,   0);
    private_nh_.param<bool>       ("Udp_hand",             udp_hand_,              false);
    private_nh_.param<bool>       ("Udp_plus_state",       udp_plus_state_,        false);
    private_nh_.param<bool>       ("Udp_plus_base",        udp_plus_base_,         false);
    private_nh_.param<int>        ("trajectory_mode",      trajectory_mode_,       0);
    private_nh_.param<int>        ("radio",                radio_,                 0);
    private_nh_.param<bool>       ("is_4th_Gen",           is_4th_Gen_,            false);
    signal(SIGINT, my_handler); 
    

    while (Arm_Socket_Start())
    {
        if(ctrl_flag)
        { 
            Arm_Socket_Close();
            Udp_Socket_Close();
            // ros::waitForShutdown();          //会导致无法退出的问题，现在直接使用shutdown退出
            ROS_INFO("Arm_Robot driver shut down!!!\n");
            ros::shutdown();  
            ctrl_flag = 0;
            return 0;
        }
        usleep(4000000);
    }

    ROS_INFO("/****************************************************************************\\n");
    std::cout << "\t\t   Connect " << RM_Joint.product_version  << " robot! \t\t" << std::endl;
    ROS_INFO("/****************************************************************************\\n");
    timer_cnt = 0;

    Udp_Setting.udp_cycle = Udp_cycle_;
    Udp_Setting.udp_port = Udp_Port_;
    Udp_Setting.udp_force_coordinate = Udp_force_coordinate;
    Udp_Setting.udp_ip = Udp_IP_;
    udp_min_interval = Udp_cycle_ / 1000.0;
    Udp_Setting.custom_set_data.aloha_state_ = false;
    Udp_Setting.custom_set_data.arm_current_status_ = false;
    Udp_Setting.custom_set_data.expand_state_ = false;
    Udp_Setting.custom_set_data.joint_speed_ = false;
    Udp_Setting.custom_set_data.lift_state_ = false;
    Udp_Setting.custom_set_data.hand_ = udp_hand_;
    Udp_Setting.custom_set_data.rm_plus_state_ = udp_plus_state_;
    Udp_Setting.custom_set_data.rm_plus_base_ = udp_plus_base_;

    Get_Arm_Software_Version();
    //ROS_INFO("11111111111111111111111111111111111111111111111111!\n");
    //std::cout << "controller_version_4 : " << controller_version_4 << std::endl;
    sensor_msgs::JointState real_joint;
    //发送规划角度，仿真真实机械臂连不上
    if(arm_dof == 6)
    {
        real_joint.name.resize(6);
        real_joint.position.resize(6);
        real_joint.name[0] = "joint1";
        real_joint.name[1] = "joint2";
        real_joint.name[2] = "joint3";
        real_joint.name[3] = "joint4";
        real_joint.name[4] = "joint5";
        real_joint.name[5] = "joint6";

        udp_real_joint.name.resize(6);
        udp_real_joint.position.resize(6);
        udp_real_joint.name[0] = "joint1";
        udp_real_joint.name[1] = "joint2";
        udp_real_joint.name[2] = "joint3";
        udp_real_joint.name[3] = "joint4";
        udp_real_joint.name[4] = "joint5";
        udp_real_joint.name[5] = "joint6";
        armState.joint.resize(6);
        Arm_State.joint.resize(6);
        joint_current.joint_current.resize(6);
        udp_joint_error_code.joint.resize(6);
        udp_joint_current.joint_current.resize(6);
        udp_joint_en_flag.joint_en_flag.resize(6);
        udp_joint_speed.joint_speed.resize(6);
        udp_joint_temperature.joint_temperature.resize(6);
        udp_joint_voltage.joint_voltage.resize(6);
    }
    else if(arm_dof == 7)
    {
        real_joint.name.resize(7);
        real_joint.position.resize(7);
        real_joint.name[0] = "joint1";
        real_joint.name[1] = "joint2";
        real_joint.name[2] = "joint3";
        real_joint.name[3] = "joint4";
        real_joint.name[4] = "joint5";
        real_joint.name[5] = "joint6";
        real_joint.name[6] = "joint7";

        udp_real_joint.name.resize(7);
        udp_real_joint.position.resize(7);
        udp_real_joint.name[0] = "joint1";
        udp_real_joint.name[1] = "joint2";
        udp_real_joint.name[2] = "joint3";
        udp_real_joint.name[3] = "joint4";
        udp_real_joint.name[4] = "joint5";
        udp_real_joint.name[5] = "joint6";
        udp_real_joint.name[6] = "joint7";
        armState.joint.resize(7);
        Arm_State.joint.resize(7);
        joint_current.joint_current.resize(7);
        udp_joint_error_code.joint.resize(7);
        udp_joint_current.joint_current.resize(7);
        udp_joint_en_flag.joint_en_flag.resize(7);
        udp_joint_speed.joint_speed.resize(7);
        udp_joint_temperature.joint_temperature.resize(7);
        udp_joint_voltage.joint_voltage.resize(7);
    }

    nh_.setCallbackQueue(&queue_others);
    // subscriber
    MoveJ_Cmd = nh_.subscribe("/rm_driver/MoveJ_Cmd", 10, MoveJ_Callback);
    MoveL_Cmd = nh_.subscribe("/rm_driver/MoveL_Cmd", 10, MoveL_Callback);
    MoveC_Cmd = nh_.subscribe("/rm_driver/MoveC_Cmd", 10, MoveC_Callback);
    JointPos_Cmd = nh_.subscribe("/rm_driver/JointPos", 10, JointPos_Callback);
    MoveJ_Fd_Custom_Cmd = nh_.subscribe("/rm_driver/MoveJ_Fd_Custom_Cmd", 10, MoveJ_Fd_Custom_Callback);
    Arm_DO_Cmd = nh_.subscribe("/rm_driver/Arm_Digital_Output", 10, Arm_DO_Callback);
    Arm_AO_Cmd = nh_.subscribe("/rm_driver/Arm_Analog_Output", 10, Arm_AO_Callback);
    Tool_DO_Cmd = nh_.subscribe("/rm_driver/Tool_Digital_Output", 10, Tool_DO_Callback);
    Tool_AO_Cmd = nh_.subscribe("/rm_driver/Tool_Analog_Output", 10, Tool_AO_Callback);
    Gripper_Cmd = nh_.subscribe("/rm_driver/Gripper_Pick", 10, Gripper_Pick_Callback);
    sub_setGripperPickOn = nh_.subscribe("/rm_driver/Gripper_Pick_On", 10, Gripper_Pick_On_Callback);
    Gripper_Set_Cmd = nh_.subscribe("/rm_driver/Gripper_Set", 10, Gripper_Set_Callback);
    Emergency_Stop = nh_.subscribe("/rm_driver/Emergency_Stop", 1, Stop_Callback);
    Joint_En = nh_.subscribe("/rm_driver/Joint_Enable", 10, Joint_Enable_Callback);
    System_En = nh_.subscribe("/rm_driver/Clear_System_Err", 10, System_Enable_Callback);
    IO_Update = nh_.subscribe("/rm_driver/IO_Update", 1, IO_Update_Callback);


    std::cout <<"is_4th_GEn:" <<is_4th_Gen_<<std::endl;
    if(is_4th_Gen_ == 0)
    {
        /****************************************modbus 模式配置（三代控制器）*********************************************/
        sub_setRS485 = nh_.subscribe("/rm_driver/Set_RS485", 10, Set_RS485_Callback);
        sub_getControllerRS485Mode = nh_.subscribe("/rm_driver/Get_Controller_RS485_Mode",10, Get_Controller_RS485_Mode_Callback);
        pub_getControllerRS485Mode_result = nh_.advertise<rm_msgs::RS485_Mode>("/rm_driver/Get_Controller_RS485_Mode_Result",10);
        sub_getToolRS485Mode = nh_.subscribe("/rm_driver/Get_Tool_RS485_Mode", 10, Get_Tool_RS485_Mode_Callback);
        pub_getToolRS485Mode_result = nh_.advertise<rm_msgs::RS485_Mode>("/rm_driver/Get_Tool_RS485_Mode_Result",10);
        sub_setModbusMode = nh_.subscribe("/rm_driver/Set_Modbus_Mode", 10, Set_Modbus_Mode_Callback);
        pub_setModbusMode_result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Modbus_Mode_Result",10);
        sub_closeModbusMode = nh_.subscribe("/rm_driver/Close_Modbus_Mode", 10, Close_Modbus_Mode_Callback);
        pub_closeModbusMode_result = nh_.advertise<std_msgs::Bool>("/rm_driver/Close_Modbus_Mode_Result",10);
        sub_setModbustcpMode = nh_.subscribe("/rm_driver/Set_Modbustcp_Mode", 10, Set_Modbustcp_Mode_Callback);
        pub_setModbustcpMode_result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Modbustcp_Mode_Result",10);
        sub_closeModbustcpMode = nh_.subscribe("/rm_driver/Close_Modbustcp_Mode", 10, Close_Modbustcp_Mode_Callback);
        pub_closeModbustcpMode_result = nh_.advertise<std_msgs::Bool>("/rm_driver/Close_Modbustcp_Mode_Result",10);

        /****************************************modbus 读写寄存器（三代控制器）*********************************************/
        sub_readCoils = nh_.subscribe("/rm_driver/Read_Coils", 10, Read_Coils_Callback);
        pub_readCoils_result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Coils_Result",10);
        sub_readMultipleCoils = nh_.subscribe("/rm_driver/Read_Multiple_Coils", 10, Read_Multiple_Coils_Callback);
        pub_readMultipleCoils_result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Multiple_Coils_Result",10);
        sub_writeSingleCoil = nh_.subscribe("/rm_driver/Write_Single_Coil", 10, Write_Single_Coil_Callback);
        pub_writeSingleCoil_result = nh_.advertise<std_msgs::Bool>("/rm_driver/Write_Single_Coil_Result",10);
        sub_writeCoils = nh_.subscribe("/rm_driver/Write_Coils", 10, Write_Coils_Callback);
        pub_writeCoils_result = nh_.advertise<std_msgs::Bool>("/rm_driver/Write_Coils_Result",10);
        sub_readInputStatus = nh_.subscribe("/rm_driver/Read_Input_Status", 10, Read_Input_Status_Callback);
        pub_readInputStatus_result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Input_Status_Result",10);
        sub_readHoldingRegisters = nh_.subscribe("/rm_driver/Read_Holding_Registers", 10, Read_Holding_Registers_Callback);
        pub_readHoldingRegisters_result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Holding_Registers_Result",10);
        sub_readMultipleHoldingRegisters = nh_.subscribe("/rm_driver/Read_Multiple_Holding_Registers", 10, Read_Multiple_Holding_Registers_Callback);
        pub_readMultipleHoldingRegisters_result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Multiple_Holding_Registers_Result",10);
        sub_writeSingleRegister = nh_.subscribe("/rm_driver/Write_Single_Register", 10, Write_Single_Register_Callback);
        pub_writeSingleRegister_result = nh_.advertise<std_msgs::Bool>("/rm_driver/Write_Single_Register_Result",10);
        sub_writeRegisters = nh_.subscribe("/rm_driver/Write_Registers", 10, Write_Registers_Callback);
        pub_writeRegisters_result = nh_.advertise<std_msgs::Bool>("/rm_driver/Write_Registers_Result",10);
        sub_readInputRegisters = nh_.subscribe("/rm_driver/Read_Input_Registers", 10, Read_Input_Registers_Callback);
        pub_readInputRegisters_result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Input_Registers_Result",10);
        sub_readMultipleInputRegisters = nh_.subscribe("/rm_driver/Read_Multiple_Input_Registers", 10, Read_Multiple_Input_Registers_Callback);
        pub_readMultipleInputRegisters_result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Multiple_Input_Registers_Result",10);



        
    }
    else if(is_4th_Gen_ == 1)
    {
        /********************************************新增Modbus TCP主站********************************************/
        Add_Modbus_Tcp_Master_cmd = nh_.subscribe("/rm_driver/Add_Modbus_Tcp_Master_cmd", 10, Add_Modbus_Tcp_Master_Callback);
        Add_Modbus_Tcp_Master_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Add_Modbus_Tcp_Master_Result", 10);
        /********************************************更新Modbus TCP主站********************************************/
        Update_Modbus_Tcp_Master_cmd = nh_.subscribe("/rm_driver/Update_Modbus_Tcp_Master_cmd", 10, Update_Modbus_Tcp_Master_Callback);
        Update_Modbus_Tcp_Master_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Update_Modbus_Tcp_Master_Result", 10);
        /********************************************删除Modbus TCP主站********************************************/
        Delete_Modbus_Tcp_Master_cmd = nh_.subscribe("/rm_driver/Delete_Modbus_Tcp_Master_cmd", 10, Delete_Modbus_Tcp_Master_Callback);
        Delete_Modbus_Tcp_Master_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Delete_Modbus_Tcp_Master_Result", 10);
        /********************************************查询指定Modbus TCP主站********************************************/
        Get_Modbus_Tcp_Master_cmd = nh_.subscribe("/rm_driver/Get_Modbus_Tcp_Master_cmd", 10, Get_Modbus_Tcp_Master_Callback);
        Get_Modbus_Tcp_Master_Result = nh_.advertise<rm_msgs::Modbustcpmasterinfo>("/rm_driver/Get_Modbus_Tcp_Master_Result", 10);
        /********************************************查询TCP主站列表********************************************/
        Get_Modbus_Tcp_Master_List_cmd = nh_.subscribe("/rm_driver/Get_Modbus_Tcp_Master_List_cmd", 10, Get_Modbus_Tcp_Master_List_Callback);
        Get_Modbus_Tcp_Master_List_Result = nh_.advertise<rm_msgs::Modbustcpmasterlist>("/rm_driver/Get_Modbus_Tcp_Master_List_Result", 10);
        /********************************************设置控制器RS485模式(四代控制器支持)********************************************/
        Set_Controller_Rs485_Mode_cmd = nh_.subscribe("/rm_driver/Set_Controller_Rs485_Mode_cmd", 10, Set_Controller_Rs485_Mode_Callback);
        Set_Controller_Rs485_Mode_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Controller_Rs485_Mode_Result", 10);
        /********************************************查询控制器RS485模式(四代控制器支持)********************************************/
        Get_Controller_Rs485_Mode_V4_cmd = nh_.subscribe("/rm_driver/Get_Controller_Rs485_Mode_V4_cmd", 10, Get_Controller_Rs485_Mode_V4_Callback);
        Get_Controller_Rs485_Mode_V4_Result = nh_.advertise<rm_msgs::RS485params>("/rm_driver/Get_Controller_Rs485_Mode_V4_Result", 10);
        /********************************************设置工具端RS485模式(四代控制器支持)********************************************/
        Set_Tool_Rs485_Mode_cmd = nh_.subscribe("/rm_driver/Set_Tool_Rs485_Mode_cmd", 10, Set_Tool_Rs485_Mode_Callback);
        Set_Tool_Rs485_Mode_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Tool_Rs485_Mode_Result", 10);
        /********************************************查询工具端RS485模式(四代控制器支持)********************************************/
        Get_Tool_Rs485_Mode_V4_cmd = nh_.subscribe("/rm_driver/Get_Tool_Rs485_Mode_V4_cmd", 10, Get_Tool_Rs485_Mode_V4_Callback);
        Get_Tool_Rs485_Mode_V4_Result = nh_.advertise<rm_msgs::RS485params>("/rm_driver/Get_Tool_Rs485_Mode_V4_Result", 10);


        /****************************************modbus 读写寄存器（四代控制器）*********************************************/
        //Modbus协议读线圈
        Read_Modbus_Coils_cmd = nh_.subscribe("/rm_driver/Read_Multiple_Coils", 10, Read_Modbus_Coils_Callback);
        Read_Modbus_Coils_Result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Multiple_Coils_Result", 10);
        //Modbus协议写线圈
        Write_Modbus_Coils_cmd = nh_.subscribe("/rm_driver/Write_Coils", 10, Write_Modbus_Coils_Callback);
        Write_Modbus_Coils_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Write_Coils_Result", 10);
        //TCP协议读离散量输入
        Read_Modbus_Input_Status_cmd = nh_.subscribe("/rm_driver/Read_Input_Status", 10, Read_Modbus_Input_Status_Callback);
        Read_Modbus_Input_Status_Result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Input_Status_Result", 10);
        //Modbus协议读保持寄存器
        Read_Modbus_Holding_Registers_cmd = nh_.subscribe("/rm_driver/Read_Multiple_Holding_Registers", 10, Read_Modbus_Holding_Registers_Callback);
        Read_Modbus_Holding_Registers_Result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Multiple_Holding_Registers_Result", 10);
        //Modbus协议写保持寄存器
        Write_Modbus_Registers_cmd = nh_.subscribe("/rm_driver/Write_Registers", 10, Write_Modbus_Registers_Callback);
        Write_Modbus_Registers_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Write_Registers_Result", 10);
        //Modbus协议读输入寄存器
        Read_Modbus_Input_Registers_cmd = nh_.subscribe("/rm_driver/Read_Multiple_Input_Registers", 10, Read_Modbus_Input_Registers_Callback);
        Read_Modbus_Input_Registers_Result = nh_.advertise<rm_msgs::Register_Data>("/rm_driver/Read_Multiple_Input_Registers_Result", 10);


        /********************************************获取关节软件版本信息********************************************/
        Get_Joint_Software_Version_Cmd = nh_.subscribe("/rm_driver/Get_Joint_Software_Version_Cmd", 10, Get_Joint_Software_Version_Callback);
        Get_Joint_Software_Version_Result = nh_.advertise<rm_msgs::Jointversion>("/rm_driver/Get_Joint_Software_Version_Result", 10);
        /********************************************获取末端接口板软件版本信息********************************************/
        Get_Tool_Software_Version_Cmd = nh_.subscribe("/rm_driver/Get_Tool_Software_Version_Cmd", 10, Get_Tool_Software_Version_Callback);
        Get_Tool_Software_Version_Result = nh_.advertise<std_msgs::String>("/rm_driver/Get_Tool_Software_Version_Result", 10);  

        /********************************************设置机械臂急停状态********************************************/
        Set_Arm_Emergency_Stop_cmd = nh_.subscribe("/rm_driver/Set_Arm_Emergency_Stop", 10, Set_Arm_Emergency_Stop_Callback);
        Set_Arm_Emergency_Stop_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Arm_Emergency_Stop_Result", 10);

        /********************************************查询轨迹列表********************************************/
        Get_Trajectory_File_List_cmd = nh_.subscribe("/rm_driver/Get_Trajectory_File_List_Cmd", 10, Get_Trajectory_File_List_Callback);
        Get_Trajectory_File_List_Result = nh_.advertise<rm_msgs::Trajectorylist>("/rm_driver/Get_Trajectory_File_List_Result", 10);
        /********************************************开始运行指定轨迹********************************************/
        Set_Run_Trajectory_cmd = nh_.subscribe("/rm_driver/Set_Run_Trajectory_Cmd", 10, Set_Run_Trajectory_Callback);
        Set_Run_Trajectory_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Run_Trajectory_Result", 10);
        /********************************************删除指定轨迹********************************************/
        Delete_Trajectory_File_cmd = nh_.subscribe("/rm_driver/Delete_Trajectory_File_Cmd", 10, Delete_Trajectory_File_Callback);
        Delete_Trajectory_File_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Delete_Trajectory_File_Result", 10);
        /********************************************保存轨迹到控制机器********************************************/
        Save_Trajectory_File_cmd = nh_.subscribe("/rm_driver/Save_Trajectory_File_Cmd", 10, Save_Trajectory_File_Callback);
        Save_Trajectory_File_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Save_Trajectory_File_Result", 10);

        /********************************************查询流程图编程状态********************************************/
        Get_Flowchart_Program_Run_State_cmd = nh_.subscribe("/rm_driver/Get_Flowchart_Program_Run_State_Cmd", 10, Get_Flowchart_Program_Run_State_Callback);
        Get_Flowchart_Program_Run_State_Result = nh_.advertise<rm_msgs::Flowchartrunstate>("/rm_driver/Get_Flowchart_Program_Run_State_Result", 10);
        /********************************************笛卡尔空间直线偏移运动********************************************/
        Movel_Offset_cmd = nh_.subscribe("/rm_driver/Movel_Offset_Cmd", 10, Movel_Offset_Callback);
        Movel_Offset_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Movel_Offset_Result", 10);
    }

    
    /********************************************获取软件版本信息********************************************/
    Get_Arm_Software_Version_Cmd = nh_.subscribe("/rm_driver/Get_Arm_Software_Version_Cmd", 10, Get_Arm_Software_Version_Callback);
    Get_Arm_Software_Version_Result = nh_.advertise<rm_msgs::Arm_Software_Version>("/rm_driver/Get_Arm_Software_Version_Result", 20);
    /********************************************udp配置参数修改控制********************************************/
    Set_Realtime_Push = nh_.subscribe("/rm_driver/Set_Realtime_Push", 10, Set_Realtime_Push_callback);
    Set_Realtime_Push_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Realtime_Push_Result", 10);
    /*******************************************udp配置参数查询控制******************************************/
    Get_Realtime_Push = nh_.subscribe("/rm_driver/Get_Realtime_Push", 10, Get_Realtime_Push_callback);
    Get_Realtime_Push_Result = nh_.advertise<rm_msgs::Set_Realtime_Push>("/rm_driver/Get_Realtime_Push_Result", 10);
    /********************************************力传感器外力数据********************************************/
    pub_ArmError = nh_.advertise<std_msgs::UInt16>("/rm_driver/ArmError", 100);
    pub_SysError = nh_.advertise<std_msgs::UInt16>("/rm_driver/SysError", 100);
    pub_UdpError = nh_.advertise<rm_msgs::Err>("/rm_driver/Error", 100);
    pub_JointErrorCode = nh_.advertise<rm_msgs::Manual_Set_Force_Pose>("/rm_driver/JointErrorCode", 100);
    /*****************************************发布当前的受力基准坐标系*******************************************/
    pub_Udp_Coordinate = nh_.advertise<std_msgs::UInt16>("/rm_driver/Udp_Coordinate", 10);

    sub_getArmStateTimerSwitch = nh_.subscribe("/rm_driver/GetArmStateTimerSwitch", 1, getArmStateTimerSwitch_Callback);

    ros::Subscriber MoveP_Fd_Cmd = nh_.subscribe("/rm_driver/MoveP_Fd_Cmd", 10, Movep_Fd_Callback);
    MoveP_Fd_Custom_Cmd = nh_.subscribe("/rm_driver/MoveP_Fd_Custom_Cmd", 10, Movep_Fd_Custom_Callback);

    sub_getCurrJointCurrent = nh_.subscribe("/rm_driver/GetCurrentJointCurrent", 10, GetCurrJointCurrent_Callback);
    sub_setJointStep = nh_.subscribe("/rm_driver/SetJointStep", 10, SetJointStep_Callback);
    sub_getTotalWorkFrame = nh_.subscribe("/rm_driver/GetTotalWorkFrame", 10, GetTotalWorkFrame_Callback);
    sub_setArmPower = nh_.subscribe("/rm_driver/SetArmPower", 10, SetArmPower_Callback);
    sub_setToolVoltage = nh_.subscribe("/rm_driver/SetToolVoltage", 10, SetToolVoltage_Callback);

    sub_setHandPosture = nh_.subscribe("/rm_driver/Hand_SetPosture", 10, SetHandPosture_Callback);
    sub_setHandSeq = nh_.subscribe("/rm_driver/Hand_SetSeq", 10, SetHandSeq_Callback);
    sub_setHandAngle = nh_.subscribe("/rm_driver/Hand_SetAngle", 10, SetHandAngle_Callback);
    sub_setHandSpeed = nh_.subscribe("/rm_driver/Hand_SetSpeed", 10, SetHandSpeed_Callback);
    sub_setHandForce = nh_.subscribe("/rm_driver/Hand_SetForce", 10, SetHandForce_Callback);
    sub_setHandFollowAngle = nh_.subscribe("/rm_driver/Hand_FollowAngle", 10, HandFollowAngle_Callback);
    sub_setHandFollowPos = nh_.subscribe("/rm_driver/Hand_FollowPos", 10, HandFollowPos_Callback);


    /***** ********************************START***************************************
     * 20210901修改: 增加对Turtle底盘的控制相关
     * 订阅控制Turtle的主题并调用TurtleCtr_Callback处理
     * *********************************************************************************/
    ROS_INFO("subscribe chassis_topic!\n");
    turtleCtrMsgSubscriber = nh_.subscribe("/chassis_topic", 10, TurtleCtr_Callback);
    /***** ********************************END****************************************/


    /***** ********************************START**************************************
     * 20220628修改: 增加MoveJ_P指令、对机械臂切换工具坐标系、切换工作坐标系、查询机械臂状态
     * 订阅对机械臂的MoveJ_P指令、对机械臂切换工具坐标系、切换工作坐标系、查询机械臂状态的话题并调用相应回调函数处理
     * 发布对机械臂切换工具坐标系、切换工作坐标系、查询机械臂状态的话题，发布返回结果
     * *********************************************************************************/
    MoveJ_P_Cmd = nh_.subscribe("/rm_driver/MoveJ_P_Cmd", 10, MoveJ_P_Callback);
    Sub_ChangeToolName = nh_.subscribe("/rm_driver/ChangeToolName_Cmd", 10, ChangeToolName_Callback);
    Sub_ChangeWorkFrame = nh_.subscribe("/rm_driver/ChangeWorkFrame_Cmd", 10, ChangeWorkFrame_Callback);
    Sub_GetArmState = nh_.subscribe("/rm_driver/GetArmState_Cmd", 10, GetArmState_Callback);
    sub_getCurrArmState = nh_.subscribe("/rm_driver/GetCurrentArmState", 10, GetCurrentArmState_Callback);

    // publisher
    ChangeTool_Name = nh_.advertise<rm_msgs::ChangeTool_State>("/rm_driver/ChangeTool_State", 10);
    ChangeWorkFrame_Name = nh_.advertise<rm_msgs::ChangeWorkFrame_State>("/rm_driver/ChangeWorkFrame_State", 10);
    ArmCurrentState = nh_.advertise<rm_msgs::Arm_Current_State>("/rm_driver/Arm_Current_State", 10);
    pub_armCurrentState = nh_.advertise<rm_msgs::ArmState>("/rm_driver/ArmCurrentState", 10);

    /***** ********************************END****************************************/

    /***** ********************************START***************************************
     * 20220808修改: 增加对机械臂的复合拖动示教、力位混合控制、结束力位混合控制
     开启透传力位混合控制补偿模式、透传力位混合补偿（角度）、透传力位混合补偿（位姿）、关闭透传力位混合控制补偿模式
     * *********************************************************************************/
    Sub_StartMultiDragTeach = nh_.subscribe("/rm_driver/StartMultiDragTeach_Cmd", 10, StartMultiDragTeach_Callback);
    Sub_StopDragTeach = nh_.subscribe("/rm_driver/StopDragTeach_Cmd", 10, StopDragTeach_Callback);
    Sub_SetForcePosition = nh_.subscribe("/rm_driver/SetForcePosition_Cmd", 10, SetForcePosition_Callback);
    Sub_StopForcePosition = nh_.subscribe("/rm_driver/StopForcePosition_Cmd", 10, StopForcePosition_Callback);

    Sub_ToGetSixForce = nh_.subscribe("/rm_driver/GetSixForce_Cmd", 10, GetSixForce_Callback);
    Sub_ClearForceData = nh_.subscribe("/rm_driver/ClearForceData_Cmd", 10, ClearForceData_Callback);
    Sub_SetForceSensor = nh_.subscribe("/rm_driver/SetForceSensor_Cmd", 10, SetForceSensor_Callback);
    Sub_ManualSetForcePose = nh_.subscribe("/rm_driver/ManualSetForcePose_Cmd", 10, ManualSetForcePose_Callback);
    Sub_StopSetForceSensor = nh_.subscribe("/rm_driver/StopSetForceSensor_Cmd", 10, StopSetForceSensor_Callback);

    Sub_GetArmJoint = nh_.subscribe("/rm_driver/GetArmJoint_Cmd", 100, GetArmJoint_Callback);

    sub_lift_setHeight = nh_.subscribe("/rm_driver/Lift_SetHeight", 10, LiftHeightCtr_Callback);
    sub_setLiftSpeed = nh_.subscribe("/rm_driver/Lift_SetSpeed", 10, LiftSpeedCtr_Callback);
    sub_getLiftState = nh_.subscribe("/rm_driver/Lift_GetState", 10, GetLiftState_Callback);

    Sub_ToGetOneForce = nh_.subscribe("/rm_driver/GetOneForce_Cmd", 10, GetOneForce_Callback);

    nh_.setCallbackQueue(&queue_forcePositionMove);
    Sub_StartForcePositionMove = nh_.subscribe("/rm_driver/StartForcePositionMove_Cmd", 10, StartForcePositionMove_Callback);
    Sub_StopForcePositionMove = nh_.subscribe("/rm_driver/StopForcePositionMove_Cmd", 10, StopForcePositionMove_Callback);
    Sub_ForcePositionMovePose = nh_.subscribe("/rm_driver/ForcePositionMovePose_Cmd", 10, ForcePositionMovePose_Callback);
    Sub_ForcePositionMovePoseCustom = nh_.subscribe("/rm_driver/ForcePositionMovePoseCustom_Cmd", 10, ForcePositionMovePoseCustom_Callback);
    Sub_ForcePositionMoveJiont = nh_.subscribe("/rm_driver/ForcePositionMoveJiont_Cmd", 10, ForcePositionMoveJiont_Callback);

    /**************************************START***************************************
     * 20211103修改: 增加对机械臂的关节示教,位置示教,姿态示教和停止示教
     * 订阅对机械臂的关节示教,位置示教和停止示教的主题并调用相应回调函数处理
     * *********************************************************************************/
    // ROS_INFO("subscribe /rm_driver/Lift_SetSpeed!\n");
    sub_setJointTeach = nh_.subscribe("/rm_driver/Arm_JointTeach", 10, JointTeach_Callback);
    sub_setPosTeach = nh_.subscribe("/rm_driver/Arm_PosTeach", 10, PosTeach_Callback);
    sub_setOrtTeach = nh_.subscribe("/rm_driver/Arm_OrtTeach", 10, OrtTeach_Callback);
    sub_setStopTeach = nh_.subscribe("/rm_driver/Arm_StopTeach", 10, StopTeach_Callback);
    /***** ********************************END****************************************/
    nh_.setCallbackQueue(&queue_armJog);
    /*************************************示教返回**********************************************/
    pub_setJointTeachResult = nh_.advertise<std_msgs::Bool>("/rm_driver/SetJointTeach_Result", 10);  //角度
    pub_setPosTeachResult = nh_.advertise<std_msgs::Bool>("/rm_driver/SetPosTeach_Result", 10);      //位置
    pub_setOrtTeachResult = nh_.advertise<std_msgs::Bool>("/rm_driver/SetOrtTeach_Result", 10);      //姿态
    pub_setStopResult = nh_.advertise<std_msgs::Bool>("/rm_driver/SetStopTeach_Result", 10);         //停止

    ros::AsyncSpinner spinner_others(4, &queue_others);
    spinner_others.start();

    ros::AsyncSpinner spinner_forcePositionMove(1, &queue_forcePositionMove);
    spinner_forcePositionMove.start();

    ros::AsyncSpinner spinner_armJog(1, &queue_armJog);
    spinner_armJog.start();

    // publisher
    pub_StartMultiDragTeach_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/StartMultiDragTeach_Result", 10);
    pub_StopDragTeach_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/StopDragTeach_Result", 10);
    pub_SetForcePosition_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/SetForcePosition_Result", 10);
    pub_StopForcePosition_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/StopForcePosition_Result", 10);

    pub_ClearForceData_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/ClearForceData_Result", 10);
    pub_ForceSensorSet_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/ForceSensorSet_Result", 10);
    pub_StopSetForceSensor_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/StopSetForceSensor_Result", 10);

    pub_StartForcePositionMove_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/StartForcePositionMove_Result", 10);
    pub_StopForcePositionMove_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/StopForcePositionMove_Result", 10);
    pub_Force_Position_State = nh_.advertise<rm_msgs::Force_Position_State>("/rm_driver/Force_Position_State", 10);
    pub_Force_Position_Move_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Force_Position_Move_Result", 10);
    
    /**************************************END****************************************/

    // publisher
    Joint_State = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
    Arm_IO_State = nh_.advertise<rm_msgs::Arm_IO_State>("/rm_driver/Arm_IO_State", 10);
    Tool_IO_State = nh_.advertise<rm_msgs::Tool_IO_State>("/rm_driver/Tool_IO_State", 10);
    Plan_State = nh_.advertise<rm_msgs::Plan_State>("/rm_driver/Plan_State", 10);
    pub_PoseState = nh_.advertise<geometry_msgs::Pose>("/rm_driver/Pose_State", 10);
    pub_HandStatus = nh_.advertise<rm_msgs::Hand_Status>("/rm_driver/Udp_Hand_Status", 1);
    pub_RmPlusState = nh_.advertise<rm_msgs::Rm_Plus_State>("/rm_driver/Udp_Plus_State", 1);
    pub_RmPlusBase = nh_.advertise<rm_msgs::Rm_Plus_Base>("/rm_driver/Udp_Plus_Base", 1);
    pub_currentJointCurrent = nh_.advertise<rm_msgs::Joint_Current>("/rm_driver/Joint_Current", 10);
    pub_liftState = nh_.advertise<rm_msgs::LiftState>("/rm_driver/LiftState", 10);
    pub_setGripperResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Gripper_Result", 10);

    pub_ArmCurrentStatus = nh_.advertise<rm_msgs::Arm_Current_Status>("/rm_driver/Udp_Arm_Current_status",10);
    pub_JointCurrent = nh_.advertise<rm_msgs::Joint_Current>("/rm_driver/Udp_Joint_Current",10);
    pub_JointEnFlag = nh_.advertise<rm_msgs::Joint_En_Flag>("/rm_driver/Udp_Joint_En_Flag",10);
    pub_JointSpeed = nh_.advertise<rm_msgs::Joint_Speed>("/rm_driver/Udp_Joint_Speed",10);
    pub_JointTemperature = nh_.advertise<rm_msgs::Joint_Temperature>("/rm_driver/Udp_Joint_Temperature",10);
    pub_JointVoltage = nh_.advertise<rm_msgs::Joint_Voltage>("/rm_driver/Udp_Joint_Voltage",10);
    pub_PoseEuler = nh_.advertise<rm_msgs::Joint_PoseEuler>("/rm_driver/Udp_Joint_PoseEuler",10);
    pub_LiftInPosition = nh_.advertise<rm_msgs::Lift_In_Position>("/rm_driver/Lift_InPosition",10);
    pub_ExpandInPosition = nh_.advertise<rm_msgs::Expand_In_Position>("/rm_driver/Expand_InPosition",10);
    /******************************************************************************************/
    pub_setLiftSpeedResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Lift_Speed_Result", 10);
    /***************************************灵巧手控制****************************************/
    pub_setHandPostureResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Hand_Posture_Result", 10);
    pub_setHandSeqResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Hand_Seq_Result", 10);
    pub_setHandHAngleResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Hand_Angle_Result", 10);
    pub_set_HandSpeedResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Hand_Speed_Result", 10);
    pub_setHandForceResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Hand_Force_Result", 10);
    pub_setHandFollowAngleResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Hand_Follow_Angle_Result", 10);
    pub_setHandFollowPosResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Hand_Follow_Pos_Result", 10);
    /**************************************机械臂电源控制******************************************/
    pub_setArmPowerResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Arm_Power_Result", 10);
    /**************************************工具端电源输出返回***************************************/
    pub_setToolVoltageResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Tool_Voltage_Result", 10);
    /**************************************工具端数字IO输出状态***************************************/
    pub_setToolDOStateResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Tool_DO_State_Result", 10);
    /**************************************设置IO输出状态****************************************/
    pub_setDOStateResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_DO_State_Result", 10);
    pub_setAOStateResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_AO_State_Result", 10);
    /**************************************轨迹急停**********************************************/
    pub_setArmStopResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Arm_Stop_Result", 10);
    /***********************************清除关节错误代码*******************************************/
    pub_Joint_Clear_Err_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Joint_Clear_Err_Result", 10);
    /***************************************使能、失能关节*******************************************/
    pub_Joint_En_State_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Joint_En_State_Result", 10);
    /***************************************清除系统错误代码*******************************************/
    pub_System_En_State_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/System_En_State_Result", 10);

    /*************************************六维力数据发布（非UDP）*******************************************/
    pub_GetSixForce = nh_.advertise<rm_msgs::Six_Force>("/rm_driver/GetSixForce", 100);
    pub_SixZeroForce = nh_.advertise<rm_msgs::Six_Force>("/rm_driver/SixZeroForce", 100);
    pub_Work_Zero_Force = nh_.advertise<rm_msgs::Six_Force>("/rm_driver/WorkZeroForce", 100);
    pub_Tool_Zero_Force = nh_.advertise<rm_msgs::Six_Force>("/rm_driver/ToolZeroForce", 100);
    /*************************************六维力数据发布（UDP）*******************************************/
    pub_UdpSixForce = nh_.advertise<rm_msgs::Six_Force>("/rm_driver/UdpSixForce", 100);
    pub_UdpSixZeroForce = nh_.advertise<rm_msgs::Six_Force>("/rm_driver/UdpSixZeroForce", 100);
    /*************************************设置末端生态协议模式*******************************************/
    sub_setRmPlusMode = nh_.subscribe("/rm_driver/Set_Rm_Plus_Mode", 10, SetRmPlusMode_Callback);
    pub_setRmPlusModeResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Rm_Plus_Mode_Result", 10);
    /*************************************查询末端生态协议模式*******************************************/
    sub_getRmPlusMode = nh_.subscribe("/rm_driver/Get_Rm_Plus_Mode", 10, GetRmPlusMode_Callback);
    pub_getRmPlusModeResult = nh_.advertise<std_msgs::UInt32>("/rm_driver/Get_Rm_Plus_Mode_Result", 10);
    /*************************************设置触觉传感器模式*******************************************/
    sub_setRmPlusTouch = nh_.subscribe("/rm_driver/Set_Rm_Plus_Touch", 10, SetRmPlusTouch_Callback);
    pub_setRmPlusTouchResult = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Rm_Plus_Touch_Result", 10);
    /*************************************查询触觉传感器模式*******************************************/
    sub_getRmPlusTouch = nh_.subscribe("/rm_driver/Get_Rm_Plus_Touch", 10, GetRmPlusTouch_Callback);
    pub_getRmPlusTouchResult = nh_.advertise<std_msgs::UInt16>("/rm_driver/Get_Rm_Plus_Touch_Result", 10);

    /**********************************V2.6.0************************************/
    /**********************************轨迹暂停************************************/
    sub_setArmPause_Cmd  = nh_.subscribe("/rm_driver/Set_Arm_Pause_Cmd", 10, SetArmPause_Callback);
    pub_setArmPause_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Arm_Pause_Result", 10);
    /**********************************暂停后恢复功能************************************/
    sub_setArmContinue_Cmd  = nh_.subscribe("/rm_driver/Set_Arm_Continue_Cmd", 10, SetArmContinue_Callback);
    pub_setArmContinue_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Arm_Continue_Result", 10);
    /**********************************扩展关节状态获取************************************/
    sub_getExpandState_Cmd = nh_.subscribe("/rm_driver/Get_Expand_State_Cmd", 10, GetExpandState_Callback);
    pub_getExpandState_Result = nh_.advertise<rm_msgs::ExpandState>("/rm_driver/Get_Expand_State_Result", 10);
    /**********************************扩展关节速度环控制************************************/
    sub_setExpandSpeed_Cmd = nh_.subscribe("/rm_driver/Set_Expand_Speed_Cmd", 10, SetExpandSpeed_Callback);
    pub_setExpandSpeed_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Expand_Speed_Result", 10);
    /**********************************扩展关节位置环控制************************************/
    sub_setExpandPos_Cmd = nh_.subscribe("/rm_driver/Set_Expand_Pos_Cmd", 10, SetExpandPos_Callback);
    pub_setExpandPos_Result = nh_.advertise<std_msgs::Bool>("/rm_driver/Set_Expand_Pos_Result", 10);
    // pub_ExpandInPosition = nh_.advertise<rm_msgs::Expand_In_Position>("/rm_driver/Lift_InPosition",10);
    /**********************************V2.6.0 end************************************/
    // timer
    State_Timer = nh_.createTimer(ros::Duration(min_interval), timer_callback);

    Fake_Socket_Heart = nh_.createTimer(ros::Duration(1), heart_callback);

    Udp_State_Timer = nh_.createTimer(ros::Duration(udp_min_interval), udp_timer_callback);

    State_Timer.stop();
    Fake_Socket_Heart.stop();
    Udp_State_Timer.stop();

    // init gripper
    RM_Joint.gripper_joint = GRIPPER_WIDTH / 2;
    
    Fake_Socket_Heart.start();

    // get robot state
    Get_Arm_Joint();

    ros::AsyncSpinner spinner(3);
    spinner.start();



    while (ros::ok())
    {
        while (realtime_arm_joint_state == true)
        {
            if((udp_failed_time>5)||(tcp_arm_joint_state == false))
            {
                break;
            }
            FD_ZERO(&fds);
            FD_SET(Arm_Socket, &fds);
            nRet = select(Arm_Socket+1, &fds, NULL, NULL, &time_out);
            // ROS_INFO("nRet data is:  %d", nRet);
            if (nRet == 0)
            { 
                break;
            }
            recv(Arm_Socket, temp, 1, 0);
            socket_buffer[buffer_cnt] = (char)temp[0];
            buffer_cnt++;

            if ((temp[0] == 0x0A) && (buffer_cnt > 1))
            {
                // ROS_INFO("recv data is:  %s", socket_buffer);
                if(buffer_cnt>500)
                {
                    // ROS_INFO("buffer_cnt data is:  %d", buffer_cnt);
                    tcp_arm_joint_state = false;
                    buffer_cnt = 0;
                    break;
                }
                msg = (byte)socket_buffer[buffer_cnt - 2];

                if (msg == 0x0D)
                {
                    res = Parser_Msg(socket_buffer);

                    switch (res)
                    {
                    // Update:2023-7-25 @HermanYe
                    // 机械臂控制器版本查询                        
                    case CTRL_VERSION:
                    {
                        if(CONTROLLER_VERSION == 1)
                        {
                            ROS_INFO("State_Timerstart()");
                            State_Timer.start();
                        }
                        else if((CONTROLLER_VERSION == 2) && (connect_udp_flage == false))
                        {
                            Udp_Set_Realtime_Push(Udp_cycle_/5, Udp_Port_, Udp_force_coordinate, Udp_IP_,Udp_Setting.custom_set_data);
                        }
                            Get_Arm_Software_Version_Result.publish(arm_version);
                        Info_Arm_Err();
                        break;
                    }
                    case GET_REALTIME_PUSH:
                        udp_set_realtime_push.cycle = Udp_Setting.udp_cycle;
                        udp_set_realtime_push.port = Udp_Setting.udp_port;
                        udp_set_realtime_push.force_coordinate = Udp_Setting.udp_force_coordinate;
                        udp_set_realtime_push.ip = Udp_Setting.udp_ip;
                        udp_set_realtime_push.hand_enable = Udp_Setting.custom_set_data.hand_;
                        udp_set_realtime_push.joint_speed_enable = Udp_Setting.custom_set_data.joint_speed_;
                        udp_set_realtime_push.joint_acc_enable = Udp_Setting.custom_set_data.joint_acc_;
                        udp_set_realtime_push.tail_end_enable = Udp_Setting.custom_set_data.tail_end_;
                        udp_set_realtime_push.lift_state_enable = Udp_Setting.custom_set_data.lift_state_;
                        udp_set_realtime_push.expand_state_enable = Udp_Setting.custom_set_data.expand_state_;
                        udp_set_realtime_push.arm_current_status_enable = Udp_Setting.custom_set_data.arm_current_status_;
                        udp_set_realtime_push.aloha_state_enable = Udp_Setting.custom_set_data.aloha_state_;
                        udp_set_realtime_push.rm_plus_state_enable = Udp_Setting.custom_set_data.rm_plus_state_;
                        udp_set_realtime_push.rm_plus_base_enable = Udp_Setting.custom_set_data.rm_plus_base_;
                        if(Udp_Port_ != Udp_Setting.udp_port)
                        {
                            Udp_Socket_Close();
                            Udp_Port_ = Udp_Setting.udp_port;
                            while (Udp_Socket_Start())
                            {
                                if(ctrl_flag)
                                { // my action when signal set it 1
                                    spinner_others.stop();
                                    spinner_forcePositionMove.stop();
                                    spinner_armJog.stop();
                                    Fake_Socket_Heart.stop();
                                    Udp_Socket_Close();
                                    Arm_Socket_Close();
                                    ROS_INFO("Arm_Robot driver shut down!!!\n");
                                    ros::shutdown();  
                                    ctrl_flag = 0;
                                    return 0;
                                }
                            usleep(1000);
                            }
                            Udp_State_Timer.start();
                            connect_udp_flage = true;
                        }
                        Get_Realtime_Push_Result.publish(udp_set_realtime_push);
                        break;
                    case ARM_JOINT_STATE:
                        real_joint.header.stamp = ros::Time::now();
                        for (i = 0; i < 6; i++)
                        {
                            real_joint.position[i] = RM_Joint.joint[i] * DEGREE_RAD;
                            // ROS_INFO("Reveive Joint State: joint[%d].position=%f",  i, real_joint.position[i]);
                        }
                        // real_joint.position[6] = RM_Joint.gripper_joint;
                        
                        Joint_State.publish(real_joint);
                        // end = std::chrono::system_clock::now();
                        // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                        // std::cout << "******joint_states duration: " << duration.count()  << " ms\n";
                        // start = end;
                        Info_Arm_Err();
                        break;
                    case SET_RM_PLUS_MODE:
                        state.data = RM_Joint.state;
                        pub_setRmPlusModeResult.publish(state);
                    case GET_RM_PLUS_MODE:
                        plus_mode_result.data = RM_Joint.mode_result;
                        pub_getRmPlusModeResult.publish(plus_mode_result);
                    case SET_RM_PLUS_TOUCH:
                        state.data = RM_Joint.state;
                        pub_setRmPlusTouchResult.publish(state);
                    case GET_RM_PLUS_TOUCH:
                        plus_touch_result.data = RM_Joint.mode_result;
                        pub_getRmPlusTouchResult.publish(plus_touch_result);
                    case ARM_JOINT_ERR:
                        Info_Joint_Err();
                        break;
                    case ARM_POSE_STATE:
                        pub_PoseState.publish(arm_pose);
                        Info_Arm_Err();
                        break;
                    case ARM_POSE_AND_JOINT_STATE:
                        real_joint.header.stamp = ros::Time::now();
                        for (i = 0; i < 6; i++)
                        {
                            real_joint.position[i] = RM_Joint.joint[i] * DEGREE_RAD;
                        }
                        Joint_State.publish(real_joint);
                        pub_PoseState.publish(arm_pose);
                        Info_Arm_Err();
                        break;
                    case ARM_IO_INPUT:
                        for (i = 0; i < 4; i++)
                        {
                            // Arm_IO.Arm_Analog_Input[i] = RM_Joint.Arm_AI[i];
                            Arm_IO.Arm_Digital_Input[i] = RM_Joint.Arm_DI[i];
                        }
                        // Arm_IO.Arm_Analog_Input[3] = RM_Joint.Arm_AI[3];
                        Arm_IO_State.publish(Arm_IO);
                        break;
                    case TOOL_IO_INPUT:
                        if(CONTROLLER_VERSION == 2)
                        {
                            for (i = 0; i < 2; i++)
                            {
                                Tool_IO.Tool_IO_Mode[i] = RM_Joint.Tool_IO_Mode[i];
                            }
                            for (i = 0; i < 2; i++)
                            {
                                Tool_IO.Tool_IO_State[i] = RM_Joint.Tool_IO_State[i];
                            }                        
                        }
                        // else if(CONTROLLER_VERSION == 1)
                        // {
                        //     for (i = 0; i < 2; i++)
                        //     {
                        //         Tool_IO.Tool_Digital_Input[i] = RM_Joint.Tool_DI[i];
                        //     }
                        //     Tool_IO.Tool_Analog_Input = RM_Joint.Tool_AI;
                        // }
                        Tool_IO_State.publish(Tool_IO);
                        break;
                    case LIFT_IN_POSITION:
                        pub_LiftInPosition.publish(lift_in_position);
                        break;
                    case EXPAND_IN_POSITION:
                        pub_ExpandInPosition.publish(expand_in_position);
                        break;
                    case PLAN_STATE_TYPE:
                        Plan.state = RM_Joint.plan_flag;
                        Plan_State.publish(Plan);
                        if (Plan.state == 0x00)
                        {
                            ROS_ERROR("Real Arm Trajectory Planning Error!");
                        }
                        break;
                    case CHANGE_TOOL_NAME:
                        ChangeTool_State.state = RM_Joint.changeTool_flag;
                        ChangeTool_Name.publish(ChangeTool_State);
                        if (ChangeTool_State.state == 0x00)
                        {
                            ROS_ERROR("Real Arm tool frame switching failed!");
                        }
                        break;
                    case CHANGE_WORK_FRAME:
                        ChangeWorkFrame_State.state = RM_Joint.ChangeWorkFrame_flag;
                        ChangeWorkFrame_Name.publish(ChangeWorkFrame_State);
                        if (ChangeWorkFrame_State.state == 0x00)
                        {
                            ROS_ERROR("Real Arm work frame switching failed!");
                        }
                        break;
                    case ARM_CURRENT_STATE:
                        for (i = 0; i < 3; i++)
                        {
                            Arm_State.joint[i] = Arm_Current_State.joint[i] / 1000.0;
                            Arm_State.Pose[i] = (Arm_Current_State.Pose[i] / 1000) / 1000.0;
                            armState.joint[i] = Arm_Current_State.joint[i];
                            armState.joint[i] = armState.joint[i] / 1000 * DEGREE_RAD;
                            // ROS_INFO("&&&&&&&&&&&joint: %f", armState.joint[i]);
                        }
                        for (i = 3; i < 6; i++)
                        {
                            Arm_State.joint[i] = Arm_Current_State.joint[i] / 1000.0;
                            Arm_State.Pose[i] = Arm_Current_State.Pose[i] / 1000.0;
                            armState.joint[i] = Arm_Current_State.joint[i];
                            armState.joint[i] = armState.joint[i] / 1000 * DEGREE_RAD;
                            Arm_State.dof = 6;
                            armState.dof = 6;
                        }
                        if(arm_dof == 7)
                        {
                            Arm_State.joint[6] = Arm_Current_State.joint[6] / 1000.0;
                            armState.joint[6] = Arm_Current_State.joint[6];
                            armState.joint[6] = armState.joint[6] / 1000 * DEGREE_RAD;
                            Arm_State.dof = 7;
                            armState.dof = 7;
                        }
                        // Arm_State.arm_err = Arm_Current_State.arm_err;
                        // Arm_State.sys_err = Arm_Current_State.sys_err;
                        Arm_State.err.resize(Arm_Current_State.error.size());
                        armState.err.resize(Arm_Current_State.error.size());
                        for(i = 0;i<Arm_Current_State.error.size();i++)
                        {
                            Arm_State.err[i] = Arm_Current_State.error[i];
                            armState.err[i] = Arm_Current_State.error[i];
                        }

                        armState.Pose.position.x = ((float)Arm_Current_State.Pose[0]) / 1000000;
                        armState.Pose.position.y = ((float)Arm_Current_State.Pose[1]) / 1000000;
                        armState.Pose.position.z = ((float)Arm_Current_State.Pose[2]) / 1000000;

                        quaternion_tf.setRPY(((float)Arm_Current_State.Pose[3]) / 1000, ((float)Arm_Current_State.Pose[4]) / 1000, ((float)Arm_Current_State.Pose[5]) / 1000);
                        quaternion_msg = tf2::toMsg(quaternion_tf); // tf类型转换为msg类型

                        armState.Pose.orientation.x = quaternion_msg.x;
                        armState.Pose.orientation.y = quaternion_msg.y;
                        armState.Pose.orientation.z = quaternion_msg.z;
                        armState.Pose.orientation.w = quaternion_msg.w;

                        ArmCurrentState.publish(Arm_State);
                        pub_armCurrentState.publish(armState);
                        break;
                    case FORCE_POSITION_STATE:
                        real_joint.header.stamp = ros::Time::now();
                        for (i = 0; i < 6; i++)
                        {
                            Force_Position_State.joint[i] = RM_Joint.joint[i];
                            real_joint.position[i] = RM_Joint.joint[i] * DEGREE_RAD;
                        }
                        Force_Position_State.dof = 6;
                        if(arm_dof == 7)
                        {
                            Force_Position_State.joint[6] = RM_Joint.joint[6];
                            real_joint.position[6] = RM_Joint.joint[6] * DEGREE_RAD;
                            Force_Position_State.dof = 7;
                        }
                        Force_Position_State.force = RM_Joint.force;
                        Force_Position_State.arm_err = arm_err;
                        pub_Force_Position_State.publish(Force_Position_State);
                        Joint_State.publish(real_joint);
                        Info_Arm_Err();
                        break;
                    case GET_SIX_FORCE:
                        Six_Force.force_Fx = RM_Joint.six_force[0];
                        Six_Force.force_Fy = RM_Joint.six_force[1];
                        Six_Force.force_Fz = RM_Joint.six_force[2];
                        Six_Force.force_Mx = RM_Joint.six_force[3];
                        Six_Force.force_My = RM_Joint.six_force[4];
                        Six_Force.force_Mz = RM_Joint.six_force[5];
                        pub_GetSixForce.publish(Six_Force);
                        zero_force.force_Fx = RM_Joint.zero_force[0];
                        zero_force.force_Fy = RM_Joint.zero_force[1];
                        zero_force.force_Fz = RM_Joint.zero_force[2];
                        zero_force.force_Mx = RM_Joint.zero_force[3];
                        zero_force.force_My = RM_Joint.zero_force[4];
                        zero_force.force_Mz = RM_Joint.zero_force[5];
                        pub_SixZeroForce.publish(zero_force);
                        work_zero_force.force_Fx = RM_Joint.work_zero_force[0];
                        work_zero_force.force_Fy = RM_Joint.work_zero_force[1];
                        work_zero_force.force_Fz = RM_Joint.work_zero_force[2];
                        work_zero_force.force_Mx = RM_Joint.work_zero_force[3];
                        work_zero_force.force_My = RM_Joint.work_zero_force[4];
                        work_zero_force.force_Mz = RM_Joint.work_zero_force[5];
                        pub_Work_Zero_Force.publish(work_zero_force);
                        tool_zero_force.force_Fx = RM_Joint.tool_zero_force[0];
                        tool_zero_force.force_Fy = RM_Joint.tool_zero_force[1];
                        tool_zero_force.force_Fz = RM_Joint.tool_zero_force[2];
                        tool_zero_force.force_Mx = RM_Joint.tool_zero_force[3];
                        tool_zero_force.force_My = RM_Joint.tool_zero_force[4];
                        tool_zero_force.force_Mz = RM_Joint.tool_zero_force[5];
                        pub_Tool_Zero_Force.publish(tool_zero_force);
                        break;
                    case GET_ONE_FORCE:
                        Six_Force.force_Fz = RM_Joint.six_force[1];
                        pub_GetSixForce.publish(Six_Force);
                        zero_force.force_Fz = RM_Joint.zero_force[1];
                        pub_SixZeroForce.publish(zero_force);
                        work_zero_force.force_Fz = RM_Joint.work_zero_force[1];
                        pub_Work_Zero_Force.publish(work_zero_force);
                        tool_zero_force.force_Fz = RM_Joint.tool_zero_force[1];
                        pub_Tool_Zero_Force.publish(tool_zero_force);
                        break;
                    case START_MULTI_DRAG_TEACH:
                        state.data = RM_Joint.state;
                        pub_StartMultiDragTeach_Result.publish(state);
                        break;
                    case STOP_DRAG_TEACH:
                        state.data = RM_Joint.state;
                        pub_StopDragTeach_Result.publish(state);
                        break;
                    case SET_FORCE_POSITION:
                        state.data = RM_Joint.state;
                        pub_SetForcePosition_Result.publish(state);
                        break;
                    case STOP_FORCE_POSITION:
                        state.data = RM_Joint.state;
                        pub_StopForcePosition_Result.publish(state);
                        break;
                    case CLEAR_FORCE_DATA:
                        state.data = RM_Joint.state;
                        pub_ClearForceData_Result.publish(state);
                        break;
                    case FORCE_SENSOR_SET:
                        state.data = RM_Joint.state;
                        pub_ForceSensorSet_Result.publish(state);
                        break;
                    case STOP_SET_FORCE_SENSOR:
                        state.data = RM_Joint.state;
                        pub_StopSetForceSensor_Result.publish(state);
                        break;
                    case START_FORCE_POSITION_MOVE:
                        state.data = RM_Joint.state;
                        pub_StartForcePositionMove_Result.publish(state);
                        break;
                    case STOP_FORCE_POSITION_MOVE:
                        state.data = RM_Joint.state;
                        pub_StopForcePositionMove_Result.publish(state);
                        break;
                    case FORCE_POSITION_MOVE:
                        state.data = RM_Joint.state;
                        pub_Force_Position_Move_Result.publish(state);
                        break;
                    case GET_CONTROLLER_RS485_MODE:
                        pub_getControllerRS485Mode_result.publish(modbus_data.get_controller_RS485_mode);
                        break;
                    case GET_TOOL_RS485_MODE:
                        pub_getToolRS485Mode_result.publish(modbus_data.get_tool_RS485_mode);
                        break;
                    case SET_MODBUS_MODE:
                        pub_setModbusMode_result.publish(modbus_data.state);
                        break;
                    case CLOSE_MODBUS_MODE:
                        pub_closeModbusMode_result.publish(modbus_data.state);
                        break;
                    case SET_MODBUSTCP_MODE:
                        pub_setModbustcpMode_result.publish(modbus_data.state);
                        break;
                    case CLOSE_MODBUSTCP_MODE:
                        pub_closeModbustcpMode_result.publish(modbus_data.state);
                        break;
                    case READ_COILS:
                        pub_readCoils_result.publish(modbus_data.read_coils);
                        break;
                    case READ_MULTIPLE_COILS:
                        pub_readMultipleCoils_result.publish(modbus_data.read_multiple_coils);
                        break;
                    case READ_INPUT_STATUS:
                        pub_readInputStatus_result.publish(modbus_data.read_input_status);
                        break;
                    case READ_HOLDING_REGISTERS:
                        pub_readHoldingRegisters_result.publish(modbus_data.read_holding_registers);
                        break;
                    case READ_MULTIPLE_HOLDING_REGISTERS:
                        pub_readMultipleHoldingRegisters_result.publish(modbus_data.read_multiple_holding_registers);
                        break;
                    case READ_INPUT_REGISTERS:
                        pub_readInputRegisters_result.publish(modbus_data.read_input_registers);
                        break;
                    case READ_MULTIPLE_INPUT_REGISTERS:
                        pub_readMultipleInputRegisters_result.publish(modbus_data.read_multiple_input_registers);
                        break;
                    case WRITE_SINGLE_COIL:
                        pub_writeSingleCoil_result.publish(modbus_data.state);
                        break;
                    case WRITE_COILS:
                        pub_writeCoils_result.publish(modbus_data.state);
                        break;
                    case WRITE_SINGLE_REGISTER:
                        pub_writeSingleRegister_result.publish(modbus_data.state);
                        break;
                    case WRITE_REGISTERS:
                        pub_writeRegisters_result.publish(modbus_data.state);
                        break;
                    case ARM_CURRENT_JOINT_CURRENT:
                        for (i = 0; i < 6; i++)
                        {
                            joint_current.joint_current[i] = RM_Joint.joint_current[i];
                        }
                        if(arm_dof == 7)
                        {
                            joint_current.joint_current[6] = RM_Joint.joint_current[6];
                        }
                        pub_currentJointCurrent.publish(joint_current);
                        break;
                    case LIFT_CURRENT_STATE:
                        liftState.height = Lift_Current_State.height;
                        liftState.current = Lift_Current_State.current;
                        liftState.err_flag = Lift_Current_State.err_flag;
                        liftState.mode = Lift_Current_State.mode;
                        pub_liftState.publish(liftState);
                        break;
                    case EXPAND_CURRENT_STATE:
                        expandState.pos = Expand_Current_State.pos;
                        expandState.current = Expand_Current_State.current;
                        expandState.err_flag = Expand_Current_State.err_flag;
                        expandState.en_flag = Expand_Current_State.en_flag;
                        expandState.mode = Expand_Current_State.mode;
                        expandState.joint_id = Expand_Current_State.joint_id;
                        pub_getExpandState_Result.publish(expandState);
                        break;
                    case SET_GRIPPER_STATE:
                        state.data = set_gripper_result;
                        pub_setGripperResult.publish(state);
                        break;
                    case SET_JOINT_TEACH:
                        state.data = RM_Joint.state;
                        pub_setJointTeachResult.publish(state);
                        break;
                    case SET_POS_TEACH:
                        state.data = RM_Joint.state;
                        pub_setPosTeachResult.publish(state);
                        break;
                    case SET_ORT_TEACH:
                        state.data = RM_Joint.state;
                        pub_setOrtTeachResult.publish(state);
                        break;
                    case SET_STOP_TEACH:
                        state.data = RM_Joint.state;
                        pub_setStopResult.publish(state);
                        break;
                    case SET_LIFT_SPEED:
                        state.data = RM_Joint.state;
                        pub_setLiftSpeedResult.publish(state);
                        break;
                    case SET_EXPAND_SPEED:
                        state.data = RM_Joint.state;
                        pub_setExpandSpeed_Result.publish(state);
                        break;
                    case SET_HAND_POSTURE:
                        state.data = RM_Joint.state;
                        pub_setHandPostureResult.publish(state);
                    case SET_HAND_SEQ:
                        state.data = RM_Joint.state;
                        pub_setHandSeqResult.publish(state);
                        break;
                    case SET_HAND_ANGLE:
                        state.data = RM_Joint.state;
                        pub_setHandHAngleResult.publish(state);
                        break;
                    case SET_HAND_SPEED:
                        state.data = RM_Joint.state;
                        pub_set_HandSpeedResult.publish(state);
                        break;
                    case SET_HAND_FORCE:
                        state.data = RM_Joint.state;
                        pub_setHandForceResult.publish(state);
                        break;
                    case HAND_FOLLOW_ANGLE:
                        state.data = RM_Joint.state;
                        pub_setHandFollowAngleResult.publish(state);
                        break;
                    case HAND_FOLLOW_POS:
                        state.data = RM_Joint.state;
                        pub_setHandFollowPosResult.publish(state);
                    case SET_ARM_POWER:
                        state.data = RM_Joint.state;
                        pub_setArmPowerResult.publish(state);
                        break;
                    case SET_TOOL_VOLTAGE:
                        state.data = RM_Joint.state;
                        pub_setToolVoltageResult.publish(state);
                        break;
                    case SET_TOOL_DO_STATE:
                        state.data = RM_Joint.state;
                        pub_setToolDOStateResult.publish(state);
                        break;
                    case SET_DO_STATE:
                        state.data = RM_Joint.state;
                        pub_setDOStateResult.publish(state);
                        break;
                    case SET_AO_STATE:
                        state.data = RM_Joint.state;
                        pub_setAOStateResult.publish(state);
                        break;
                    case SET_ARM_STOP:
                        state.data = RM_Joint.state;
                        pub_setArmStopResult.publish(state);
                        break;
                    case SET_JOINT_CLEAR_ERR:
                        state.data = RM_Joint.state;
                        pub_Joint_Clear_Err_Result.publish(state);
                        break;
                    case SET_JOINT_EN_STATE:
                        state.data = RM_Joint.state;
                        pub_Joint_En_State_Result.publish(state);
                    case SET_SYSTEM_EN_STATE:
                        state.data = RM_Joint.state;
                        pub_System_En_State_Result.publish(state);
                    case SET_ARM_CONTINUE:
                        state.data = RM_Joint.state;
                        pub_setArmContinue_Result.publish(state);
                    case SET_ARM_PAUSE:
                        state.data = RM_Joint.state;
                        pub_setArmPause_Result.publish(state);
                    case SET_REALTIME_PUSH:
                        state.data = RM_Joint.state;
                        
                        if((Udp_Port_ != Udp_Setting.udp_port)||(connect_udp_flage == false))
                        {
                            Udp_Port_ = Udp_Setting.udp_port;
                            // set_realtime_push_flag = true;
                            Udp_Socket_Close();
                            if(connect_udp_flage == true)
                            {
                                ROS_INFO("restart connect!!!!");
                            }
                            while (Udp_Socket_Start())
                            {
                                if(ctrl_flag)
                                { // my action when signal set it 1
                                    spinner_others.stop();
                                    spinner_forcePositionMove.stop();
                                    spinner_armJog.stop();
                                    Fake_Socket_Heart.stop();
                                    Udp_Socket_Close();
                                    Arm_Socket_Close();
                                    ROS_INFO("Arm_Robot driver shut down!!!\n");
                                    ros::shutdown();  
                                    ctrl_flag = 0;
                                    return 0;
                                }
                            usleep(1000);
                            }
                            Udp_State_Timer.start();
                            connect_udp_flage = true;
                        }
                        Set_Realtime_Push_Result.publish(state);
                        break;
                    case SET_ARM_EMERGENCY_STOP:
                        state.data = RM_Joint.state;
                        Set_Arm_Emergency_Stop_Result.publish(state);
                        break;
                    case GET_TRAJECTORY_FILE_LIST:
                        Get_Trajectory_File_List_Result.publish(trajectory_list);
                        break;
                    case SET_RUN_TRAJECTORY:
                        state.data = RM_Joint.state;
                        Set_Run_Trajectory_Result.publish(state);
                        break;
                    case DELETE_TRAJECTORY_FILE:
                        state.data = RM_Joint.state;
                        Delete_Trajectory_File_Result.publish(state);
                        break;
                    case SAVE_TRAJECTORY_FILE:
                        state.data = RM_Joint.state;
                        Save_Trajectory_File_Result.publish(state);
                        break;
                    case GET_FLOWCHART_PROGRAM_RUN_STATE:
                        Get_Flowchart_Program_Run_State_Result.publish(flowchart_runstate);
                        break;
                    case MOVEL_OFFSET:
                        state.data = RM_Joint.state;
                        Movel_Offset_Result.publish(state);
                        break;
                    // case ARM_VERSION:
                    //     if(controller_version_4 == 4)
                    //         Get_Arm_Software_Version_v4_Result.publish(arm_software_info_v4);
                    //     break;
                    case JOINT_VERSION:
                        Get_Joint_Software_Version_Result.publish(joint_software_version);
                        break;
                    case TOOL_VERSION:
                        Get_Tool_Software_Version_Result.publish(tool_software_version);
                        break;  
                    case ADD_MODBUS_TCP_MASTER:
                        state.data = RM_Joint.state;
                        Add_Modbus_Tcp_Master_Result.publish(state);
                        break;
                    case UPDATE_MODBUS_TCP_MASTER:
                        state.data = RM_Joint.state;
                        Update_Modbus_Tcp_Master_Result.publish(state);
                        break;
                    case DELETE_MODBUS_TCP_MASTER:
                        state.data = RM_Joint.state;
                        Delete_Modbus_Tcp_Master_Result.publish(state);
                        break;
                    case GET_MODBUS_TCP_MASTER:
                        Get_Modbus_Tcp_Master_Result.publish(modbustcp_master_info);
                        break;    
                    case GET_MODBUS_TCP_MASTER_LIST:
                        Get_Modbus_Tcp_Master_List_Result.publish(modbustcp_master_list);
                        break;    
                    case SET_CONTROLLER_RS485_MODE:
                        state.data = RM_Joint.state;
                        Set_Controller_Rs485_Mode_Result.publish(state);
                        break;   
                    case GET_CONTROLLER_RS485_MODE_V4:
                        Get_Controller_Rs485_Mode_V4_Result.publish(get_modbus_mode);
                        break;   
                    case SET_TOOL_RS485_MODE:
                        state.data = RM_Joint.state;
                        Set_Tool_Rs485_Mode_Result.publish(state);
                        break;
                    case GET_TOOL_RS485_MODE_V4:
                        Get_Tool_Rs485_Mode_V4_Result.publish(get_modbus_mode);
                        break;   
                    case MODBUS_READ:
                        //printf("Modbus_Read_Case: %d\n", Modbus_Read_Case);
                        if(Modbus_Read_Case == 1)Read_Modbus_Coils_Result.publish(read_modbus_data);
                        if(Modbus_Read_Case == 2)Read_Modbus_Input_Status_Result.publish(read_modbus_data);
                        if(Modbus_Read_Case == 3)Read_Modbus_Holding_Registers_Result.publish(read_modbus_data);
                        if(Modbus_Read_Case == 4)Read_Modbus_Input_Registers_Result.publish(read_modbus_data);
                        break;  
                    case MODBUS_WRITE:
                        state.data = RM_Joint.state;
                        if(Modbus_Write_Case == 1)Write_Modbus_Coils_Result.publish(state);
                        if(Modbus_Write_Case == 2)Write_Modbus_Registers_Result.publish(state);
                        break;                
                    default:
                        break;
                    }
                    buffer_cnt = 0;
                    memset(socket_buffer, 0, sizeof(socket_buffer));
                    break;
                }
            }
            else if (buffer_cnt > 500)
            {
                buffer_cnt = 0;
                memset(socket_buffer, 0, sizeof(socket_buffer));
                break;
            }
            
        }

    /*
    *作用：当断开连接时关闭所有不需要的线程.
    */
    if((connect_status==0) && (last_connect_status==1))
    { 
        ROS_INFO("RM_Robot driver lines stop!!!\n");
        spinner.stop();
        spinner_others.stop();
        spinner_forcePositionMove.stop();
        spinner_armJog.stop();
        if(CONTROLLER_VERSION == 1)
        {State_Timer.stop();}
        else 
        {Udp_State_Timer.stop();}
        last_connect_status = 0;
    }
    /*
    *作用：当接时开启所有关闭的线程.
    */
    else if((connect_status==1) && (last_connect_status==0))
    {
        spinner_others.start();
        spinner_forcePositionMove.start();
        spinner_armJog.start();
        spinner.start();
        if(CONTROLLER_VERSION == 2)
        {Udp_State_Timer.start();}
        else
        {State_Timer.start();}
        last_connect_status = 1;
        ROS_INFO("RM_Robot driver lines start!!!\n");
        ROS_INFO("Connect %s robot!\n",RM_Joint.product_version.c_str());
    }

    /*
    *作用当ctrl+c执行时，会调用下面的语句
    */
    if(ctrl_flag||(ros_shutdown_flage==1))
    { // my action when signal set it 1
        if(connect_udp_flage == true)
        {
            if(ros_shutdown_flage==1)
            {
                spinner_others.stop();
                spinner_forcePositionMove.stop();
                spinner_armJog.stop();
                spinner.stop();
            }
                Fake_Socket_Heart.stop();
                Arm_Socket_Close();
                if(CONTROLLER_VERSION == 1)
                State_Timer.stop();
                if(CONTROLLER_VERSION == 2)
                {
                    Udp_State_Timer.stop();
                    Udp_Socket_Close();
                }
        }
        ROS_INFO("RM_Robot driver shut down!\n");
        // ros::waitForShutdown();
        ros::shutdown();
        return 0;
        
    }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

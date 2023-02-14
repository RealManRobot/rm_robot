#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <mutex>
// messgae
#include <rm_msgs/Arm_Analog_Output.h>
#include <rm_msgs/Arm_Digital_Output.h>
#include <rm_msgs/Arm_IO_State.h>
#include <rm_msgs/Gripper_Pick.h>
#include <rm_msgs/Gripper_Set.h>
#include <rm_msgs/Joint_Enable.h>
#include <rm_msgs/JointPos.h>
#include <rm_msgs/MoveC.h>
#include <rm_msgs/MoveJ.h>
#include <rm_msgs/MoveL.h>
#include <rm_msgs/MoveJ_P.h>
#include <rm_msgs/Tool_Analog_Output.h>
#include <rm_msgs/Tool_Digital_Output.h>
#include <rm_msgs/Tool_IO_State.h>
#include <rm_msgs/Plan_State.h>
#include <rm_msgs/ChangeTool_Name.h>
#include <rm_msgs/ChangeTool_State.h>
#include <rm_msgs/ChangeWorkFrame_State.h>
#include <rm_msgs/ChangeWorkFrame_Name.h>
#include <rm_msgs/Arm_Current_State.h>
#include <rm_msgs/GetArmState_Command.h>
#include <rm_msgs/Stop.h>
#include <rm_msgs/IO_Update.h>
/***** ********************************START***************************************
 * 20210901修改: 增加对Turtle底盘的控制相关
 * *********************************************************************************/
#include <rm_msgs/Turtle_Driver.h>
#include <std_msgs/String.h>
/***** ********************************END****************************************/

/***** ********************************START***************************************
 * 20211103修改: 增加对机械臂的关节示教,位置示教,姿态示教和示教停止
 * *********************************************************************************/
#include <rm_msgs/Joint_Teach.h>
#include <rm_msgs/Pos_Teach.h>
#include <rm_msgs/Ort_Teach.h>
#include <rm_msgs/Stop_Teach.h>
/***** ********************************END****************************************/

/***** ********************************START***************************************
 * 20220808修改: 增加对机械臂的复合拖动示教、力位混合控制、结束力位混合控制
 * *********************************************************************************/
#include <std_msgs/Empty.h>
#include <rm_msgs/Start_Multi_Drag_Teach.h>
#include <rm_msgs/Set_Force_Position.h>

#include <rm_msgs/Force_Position_Move_Joint.h>
#include <rm_msgs/Force_Position_Move_Pose.h>
#include <rm_msgs/Force_Position_State.h>

#include <rm_msgs/Six_Force.h>
#include <rm_msgs/Manual_Set_Force_Pose.h>
/***** ********************************END****************************************/
#include <std_msgs/Bool.h>
#include <rm_msgs/CartePos.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <rm_msgs/Lift_Height.h>
#include <rm_msgs/Lift_Speed.h>
#include <ros/callback_queue.h>
#include <rm_msgs/Joint_Current.h>
#include <rm_msgs/Joint_Step.h>
#include <rm_msgs/ArmState.h>
#include <std_msgs/Byte.h>
#include <rm_msgs/Hand_Angle.h>
#include <rm_msgs/Hand_Force.h>
#include <rm_msgs/Hand_Posture.h>
#include <rm_msgs/Hand_Seq.h>
#include <rm_msgs/Hand_Speed.h>
#include <rm_msgs/LiftState.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "cJSON.h"

#ifdef __cplusplus
}
#endif

//////////////////////////////////////////////////////////////////////////////////
//睿尔曼智能科技有限公司        Author:Dong Qinpeng
//创建日期:20200518
//版本：V1.0.3
//版权所有，盗版必究。
// Copyright(C) 睿尔曼智能科技有限公司
// All rights reserved
//文档说明：该文档定义了机械臂接口函数的实现方式
//////////////////////////////////////////////////////////////////////////////////
#define RAD_DEGREE 57.2958
#define DEGREE_RAD 0.01745

// Gripper open width
#define GRIPPER_WIDTH 0.07 // Unit:m
#define GRIPPER_SCALE 1000

//系统初始化错误代码
#define SYS_OK 0x0000               //系统运行正常
#define TEMP_SENSOR_ERR 0x0001      //数字温度传感器初始化错误
#define POS_SENSOR_ERR 0x0002       //九轴数字传感器初始化错误
#define SD_CARD_ERR 0x0003          // SD卡初始化错误
#define NAND_FLASH_ERR 0x0004       // NAND FLASH初始化错误
#define BLUE_TEETH_ERR 0x0005       //蓝牙设备初始化错误
#define RM58S_INIT_ERR 0x0006       // RM58SWIFI模块初始化错误
#define CTRL_SYS_COM_ERR 0x0007     //实时层系统未按时上传数据
#define CTRL_SYS_INIT_ERR 0x0008    //实时层系统初始化错误
#define STATE_QUEUE_INIT_ERR 0x0009 //上传队列初始化错误
#define ZLAN1003_INIT_ERR 0x0010    // ZLAN1003初始化错误
#define TEMPERATURE_INIT_ERR 0x0011 //温度传感器初始化错误
#define DA_SENSOR_INIT_ERR 0x0012   // DA芯片初始化错误

//系统运行中DA错误
#define ARM_POSE_FALL 0x5001       //机械臂底座倾倒
#define POSE_SENSOR_ERR 0x5002     //九轴数据错误
#define SYS_TEMPER_ERR 0x5003      //控制器过温
#define TEMPER_SENSOR_ERR 0x5004   //温度传感器数据错误
#define SYS_CURRENT_OVELOAD 0x5005 //控制器过流
#define SYS_CURRENT_UNDER 0x5006   //控制器欠流
#define SYS_VOLTAGE_OVELOAD 0x5007 //控制器过压
#define SYS_VOLTAGE_UNDER 0x5008   //控制器欠压
#define CTRL_SYS_LOSS_ERR 0x5009   //实时层无法通信

//实时层错误
#define COMM_ERR 0x1001        //通讯2S中断
#define JOINT_LIMIT_ERR 0x1002 //目标角度超过关节限位
#define INVERSE_KM_ERR 0x1003  //运动学逆解错误
#define M4_CTRL_ERR 0x1004     // M4内核通信错误
#define CAN_INIT_ERR 0x1005    // CAN外设初始化失败
#define APP_BOARD_LOSS 0x1006  //接口板无法通信
#define QUEUE_INIT_ERR 0x4001  //轨迹规划点队列无法创建

//机械臂关节错误类型
#define ERR_MASK_OK 0X0000            // Normal state
#define ERR_MASK_OVER_CURRENT 0X0001  //电机电流超过安全范围
#define ERR_MASK_OVER_VOLTAGE 0X0002  //系统电压超过安全范围
#define ERR_MASK_UNDER_VOLTAGE 0X0004 //系统电压低于安全范围
#define ERR_MASK_OVER_TEMP 0X0008     //温度过高
#define ERR_MASK_HALL 0X0010          //霍尔错误
#define ERR_MASK_ENC 0X0020           //编码器出错
#define ERR_MASK_POS_TRACK 0X0040     //位置误差跟踪超限保护
#define ERR_MASK_CUR_ON 0X0080        //上电时电流传感器检测错误
#define ERR_MASK_TEMP 0X0100          //温度传感器出错标志
#define ERR_MASK_TAG_POS 0X0200       // 目标位置超限
#define ERR_MASK_DRV8320 0x0400       // DRV8320错误
#define JOINT_CAN_LOSE_ERR 0x0800     // 关节丢帧

//系统错误代码
#define ARM_OK 0x0000                       //系统正常
#define ARM_ERR_JOINT_COMMUNICATE 0x1001    //关节通信异常
#define ARM_ERR_TARGET_ANGLE_OVERRUN 0x1002 //目标角度超过限位
#define ARM_ERR_UNREACHABLE 0x1003          //该处不可达，为奇异点
#define ARM_ERR_KERNEL_COMMUNICATE 0x1004   //实时内核通信错误
#define ARM_ERR_JOINT_BUS 0x1005            //关节通信总线错误
#define ARM_ERR_PLAN_KERNEL 0x1006          //规划层内核错误
#define ARM_ERR_JOINT_OVERSPEED 0x1007      //关节超速
#define ARM_ERR_TIP_BOARD_CONNECT 0x1008    //末端接口板无法连接
#define ARM_ERR_JOINT_LOCK 0x100B           //关节抱闸未打开
#define ARM_ERR_DRAG_TEACH_OVERSPEED 0x100C //拖动示教时超速
#define ARM_ERR_CRASH 0x100D                //机械臂发生碰撞
#define ARM_ERR_NO_WCS 0x100E               //无该工作坐标系
#define ARM_ERR_NO_TCS 0x100F               //无该工具坐标系
#define ARM_ERR_JOINT_DISENABLE 0x1010      //关节发生掉使能错误

typedef unsigned char byte;

//位姿结构体
typedef struct
{
    //位置
    float px;
    float py;
    float pz;
    //欧拉角
    float rx;
    float ry;
    float rz;
} POSE;

typedef struct
{
    int16_t height;    //当前高度
    int16_t current;   //当前电流
    uint16_t err_flag; //驱动错误代码
    byte mode;         //当前升降状态
} LIFT_STATE;
LIFT_STATE Lift_Current_State;

//机械臂状态结构体，用于查询机械臂状态
typedef struct
{
    int32_t Pose[6];  //位姿
    int32_t joint[6]; //关节角度
    uint16_t arm_err; //机械臂错误代码
    uint16_t sys_err; //控制器错误代码
} ARM_STATE;
ARM_STATE Arm_Current_State;

//机械臂状态参数
typedef struct
{
    float joint[6];       //关节角度
    uint16_t err_flag[6]; //关节错误代码
    bool Arm_DI[3];
    float Arm_AI[4];
    bool Tool_DI[2];
    float Tool_AI;
    bool plan_flag;
    bool changeTool_flag;      //切换工具坐标系
    bool ChangeWorkFrame_flag; //切换工作坐标系
    float gripper_joint;       // gripper
    float force;
    float six_force[6];
    bool state; //命令执行状态
    float joint_current[6];
} JOINT_STATE;
JOINT_STATE RM_Joint;

uint16_t arm_err = 0; //系统错误代码
bool set_gripper_result = false;

geometry_msgs::Pose arm_pose;

ros::Subscriber sub_setToolVoltage;
ros::Subscriber sub_getCurrArmState;
ros::Subscriber sub_getCurrJointCurrent;
ros::Subscriber sub_setJointStep;
ros::Subscriber sub_getTotalWorkFrame;
ros::Subscriber sub_setArmPower;
ros::Subscriber sub_getLiftState;

/***** ********************************START***************************************
 * 20210901修改: 增加对Turtle底盘的控制相关
 * *********************************************************************************/
ros::Subscriber turtleCtrMsgSubscriber;
/***** ********************************END****************************************/

/***** ********************************START***************************************
 * 20211103修改: 增加对机械臂的关节示教,位置示教,姿态示教和示教停止
 * *********************************************************************************/
ros::Subscriber sub_setJointTeach, sub_setPosTeach, sub_setOrtTeach, sub_setStopTeach;
/***** ********************************END****************************************/

//升降机构的速度开环控制、升降机构的高度控制
ros::Subscriber sub_setLiftSpeed, sub_lift_setHeight;

ros::Subscriber sub_setHandPosture, sub_setHandSeq, sub_setHandAngle, sub_setHandSpeed, sub_setHandForce;

// subscriber
ros::Subscriber MoveJ_Cmd, MoveJ_P_Cmd, MoveL_Cmd, MoveC_Cmd, JointPos_Cmd, Arm_DO_Cmd, Arm_AO_Cmd, Tool_DO_Cmd, Tool_AO_Cmd, Gripper_Cmd, Gripper_Set_Cmd, Emergency_Stop, Joint_En, IO_Update, Sub_ChangeToolName, Sub_ChangeWorkFrame, Sub_GetArmState, sub_getArmStateTimerSwitch, Sub_StartMultiDragTeach, Sub_StopDragTeach, Sub_SetForcePosition, Sub_StopForcePostion, Sub_StartForcePositionMove, Sub_StopForcePositionMove, Sub_ForcePositionMovePose, Sub_ForcePositionMoveJiont, Sub_ToGetSixForce, Sub_ClearForceData, Sub_SetForceSensor, Sub_ManualSetForcePose, Sub_StopSetForceSensor, Sub_GetArmJoint;

// publisher
ros::Publisher Joint_State, Arm_IO_State, Tool_IO_State, Plan_State, ChangeTool_Name, ChangeWorkFrame_Name, ArmCurrentState, pub_Force_Position_State, pub_GetSixForce, pub_StartMultiDragTeach_result, pub_StopDragTeach_result, pub_SetForcePosition_result, pub_StopForcePostion_result, pub_ClearForceData_result, pub_ForceSensorSet_result, pub_StopSetForceSensor_result, pub_StartForcePositionMove_result, pub_StopForcePositionMove_result, pub_Force_Position_Move_result, pub_PoseState;
ros::Publisher pub_currentJointCurrent;
ros::Publisher pub_armCurrentState;
ros::Publisher pub_liftState;
ros::Publisher pub_setGripperResult;

// std::mutex mutex;

// timer
ros::Timer State_Timer;
int timer_cnt = 0;

bool startMulitiDragTeach = false;

#define ARM_JOINT_STATE 0x01
#define ARM_JOINT_ERR 0x02
#define ARM_IO_INPUT 0x03
#define TOOL_IO_INPUT 0x04
#define PLAN_STATE_TYPE 0x05
#define CHANGE_TOOL_NAME 0x06
#define CHANGE_WORK_FRAME 0x07
#define ARM_CURRENT_STATE 0x08
#define FORCE_POSITION_STATE 0x09
#define GET_SIX_FORCE 0x10
#define START_MULTI_DRAG_TEACH 0x11
#define SET_FORCE_POSITION 0x12
#define STOP_FORCE_POSTION 0x13
#define CLEAR_FORCE_DATA 0x14
#define FORCE_SENSOR_SET 0x15
#define STOP_SET_FORCE_SENSOR 0x16
#define STOP_FORCE_POSITION_MOVE 0x17
#define START_FORCE_POSITION_MOVE 0x18
#define FORCE_POSITION_MOVE 0x19
#define STOP_DRAG_TEACH 0x1A
#define ARM_POSE_STATE 0x1B
#define ARM_POSE_AND_JOINT_STATE 0x1C
#define ARM_CURRENT_JOINT_CURRENT 0x1D
#define LIFT_CURRENT_STATE 0x1E
#define SET_GRIPPER_STATE 0x1F


float min_interval = 0.02; //透传周期,单位:秒

int Arm_Socket;                      //机械臂网络通信套接字
const char *Arm_IP = "192.168.1.18"; //机械臂IP地址
int Arm_Port = 8080;                 //机械臂端口地址
std::mutex send_mutex;

// send 套壳
ssize_t package_send(int Arm_Socket, const void *buffer, size_t buffer_size, int flage)
{
    send_mutex.lock();
    // ros::Duration(0.05).sleep();
    ssize_t res = send(Arm_Socket, buffer, buffer_size, flage);
    ros::Duration(min_interval).sleep();
    send_mutex.unlock();
    return res;
}

//连接机械臂网络
int Arm_Socket_Start(void)
{
    Arm_Socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (Arm_Socket < 0)
    {
        return 2;
    }

    struct sockaddr_in serAddr;
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(Arm_Port);
    serAddr.sin_addr.s_addr = inet_addr(Arm_IP);
    if (connect(Arm_Socket, (struct sockaddr *)&serAddr, sizeof(serAddr)) < 0)
    {
        close(Arm_Socket);
        return 3;
    }

    //    //设置接收函数的超时时间
    //    struct  timeval timeout = {timeout_s, timeout_ms};
    //    setsockopt(Arm_Socket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));

    return 0;
}

//关闭机械臂网络
void Arm_Socket_Close()
{
    close(Arm_Socket);
}

//清理socket接收缓存内的数据
void Arm_Socket_Buffer_Clear()
{
    struct timeval time_out;
    time_out.tv_sec = 0;
    time_out.tv_usec = 0;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(Arm_Socket, &fds);
    int nRet;
    char temp[2];

    memset(temp, 0, sizeof(temp));

    while (1)
    {
        nRet = select(FD_SETSIZE, &fds, NULL, NULL, &time_out);
        if (nRet == 0)
            break;
        recv(Arm_Socket, temp, 1, 0);
    }
}

// 获取升降机构状态
int Get_Lift_State(void)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_lift_state");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    send_mutex.lock();
    ros::Duration(0.1).sleep();
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    ros::Duration(0.1).sleep();
    send_mutex.unlock();
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Set_Tool_Voltage_Cmd(byte state)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_tool_voltage");
    cJSON_AddNumberToObject(root, "voltage_type", state);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

// test
int Set_Arm_Power_Cmd(byte state)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_arm_power");
    cJSON_AddNumberToObject(root, "arm_power", state);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}


int Get_Total_Work_Frame(void)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_total_work_frame");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    send_mutex.lock();
    ros::Duration(0.1).sleep();
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    ros::Duration(0.1).sleep();
    send_mutex.unlock();
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送设置灵巧手手势的指令
int SetHandPostureCmd(uint16_t posture_num)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_hand_posture");
    cJSON_AddNumberToObject(root, "posture_num", posture_num);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送设置灵巧手动作序列的指令
int SetHandSeqCmd(uint16_t seq_num)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_hand_seq");
    cJSON_AddNumberToObject(root, "seq_num", seq_num);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送设置灵巧手关节速度的指令
int SetHandSpeedCmd(uint16_t hand_speed)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_hand_speed");
    cJSON_AddNumberToObject(root, "hand_speed", hand_speed);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送设置灵巧手关节力阈值的指令
int SetHandForceCmd(uint16_t hand_force)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_hand_force");
    cJSON_AddNumberToObject(root, "hand_force", hand_force);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送设置灵巧手各自由度角度的命令
int SetHandAngle(int16_t *hand_angle)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    cJSON_AddNumberToObject(array, "test", hand_angle[0]);
    cJSON_AddNumberToObject(array, "test", hand_angle[1]);
    cJSON_AddNumberToObject(array, "test", hand_angle[2]);
    cJSON_AddNumberToObject(array, "test", hand_angle[3]);
    cJSON_AddNumberToObject(array, "test", hand_angle[4]);
    cJSON_AddNumberToObject(array, "test", hand_angle[5]);

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_hand_angle");
    cJSON_AddItemToObject(root, "hand_angle", array);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Set_Joint_Step(uint8_t num, float angle, byte v)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    cJSON_AddNumberToObject(array, "test", num);
    cJSON_AddNumberToObject(array, "test", (int)(angle * 1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_joint_step");
    cJSON_AddItemToObject(root, "joint_step", array);
    cJSON_AddNumberToObject(root, "v", v);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//查询关节当前电流
int Get_Current_Joint_Current(void)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_current_joint_current");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    send_mutex.lock();
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    send_mutex.unlock();
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Movep_CANFD(POSE pose)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    //位置数据，0.001mm
    cJSON_AddNumberToObject(array, "test", (int)(pose.px * 1000000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.py * 1000000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.pz * 1000000));
    //姿态数据，0.001rad
    cJSON_AddNumberToObject(array, "test", (int)(pose.rx * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.ry * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.rz * 1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movep_canfd");
    cJSON_AddItemToObject(root, "pose", array);

    data = cJSON_Print(root);
    // ROS_INFO("Movep_CANFD:%s\n",data);

    sprintf(buffer, "%s\r\n", data);
    send_mutex.lock();
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    send_mutex.unlock();
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }

    return 0;
}

//获取机械臂关节错误代码
int Get_Joint_Err_Flag(void)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_joint_err_flag");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送控制升降机构到指定高度的指令
int SetLiftSpeedCmd(int16_t speed)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_lift_speed");
    cJSON_AddNumberToObject(root, "speed", speed);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    send_mutex.lock();
    ros::Duration(0.1).sleep();
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    ros::Duration(0.1).sleep();
    send_mutex.unlock();
    // res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送控制升降机构到指定高度的指令
int Lift_SetHeightCmd(int16_t height, int16_t speed)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_lift_height");
    cJSON_AddNumberToObject(root, "height", height);
    cJSON_AddNumberToObject(root, "speed", speed);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    send_mutex.lock();
    ros::Duration(0.1).sleep();
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    ros::Duration(0.1).sleep();
    send_mutex.unlock();
    // res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

/***** ********************************START***************************************
 * 20210901修改: 增加对Turtle底盘的控制相关
 * *********************************************************************************/
//发送控制Turtle底盘运动的指令
int SendTurtleCtrCmd(std::string message_type, std::string robot_mac_address, float vx, float vy, float vtheta)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "message_type", message_type.c_str());
    cJSON_AddStringToObject(root, "robot_mac_address", robot_mac_address.c_str());
    cJSON_AddNumberToObject(root, "vx", vx);
    cJSON_AddNumberToObject(root, "vy", vy);
    cJSON_AddNumberToObject(root, "vtheta", vtheta);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}
/***** ********************************END****************************************/

//关节空间规划
int Movej_Cmd(float *joint, byte v)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    cJSON_AddNumberToObject(array, "test", (int)(joint[0] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[1] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[2] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[3] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[4] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[5] * 1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movej");
    cJSON_AddItemToObject(root, "joint", array);
    cJSON_AddNumberToObject(root, "v", v);
    cJSON_AddNumberToObject(root, "r", (int)(r * 1000));

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//笛卡尔空间直线规划
int Movel_Cmd(POSE pose, byte v)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    //位置数据，0.001mm
    cJSON_AddNumberToObject(array, "test", (int)(pose.px * 1000000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.py * 1000000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.pz * 1000000));
    //姿态数据，0.001rad
    cJSON_AddNumberToObject(array, "test", (int)(pose.rx * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.ry * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.rz * 1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movel");
    cJSON_AddItemToObject(root, "pose", array);
    cJSON_AddNumberToObject(root, "v", v);
    cJSON_AddNumberToObject(root, "r", (int)(r * 1000));

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//笛卡尔空间直线规划
int Movec_Cmd(POSE pose_via, POSE pose_to, byte v)
{
    cJSON *root, *array1, *array2, *pose_json;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    int loop = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    pose_json = cJSON_CreateObject();
    array1 = cJSON_CreateArray();
    array2 = cJSON_CreateArray();
    //数组加入中间点数据
    //位置数据，0.001mm
    cJSON_AddNumberToObject(array1, "test", (int)(pose_via.px * 1000000));
    cJSON_AddNumberToObject(array1, "test", (int)(pose_via.py * 1000000));
    cJSON_AddNumberToObject(array1, "test", (int)(pose_via.pz * 1000000));
    //姿态数据，0.001rad
    cJSON_AddNumberToObject(array1, "test", (int)(pose_via.rx * 1000));
    cJSON_AddNumberToObject(array1, "test", (int)(pose_via.ry * 1000));
    cJSON_AddNumberToObject(array1, "test", (int)(pose_via.rz * 1000));

    //数组加入终点数据
    //位置数据，0.001mm
    cJSON_AddNumberToObject(array2, "test", (int)(pose_to.px * 1000000));
    cJSON_AddNumberToObject(array2, "test", (int)(pose_to.py * 1000000));
    cJSON_AddNumberToObject(array2, "test", (int)(pose_to.pz * 1000000));
    //姿态数据，0.001rad
    cJSON_AddNumberToObject(array2, "test", (int)(pose_to.rx * 1000));
    cJSON_AddNumberToObject(array2, "test", (int)(pose_to.ry * 1000));
    cJSON_AddNumberToObject(array2, "test", (int)(pose_to.rz * 1000));

    cJSON_AddItemToObject(pose_json, "pose_via", array1);
    cJSON_AddItemToObject(pose_json, "pose_to", array2);

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movec");
    cJSON_AddItemToObject(root, "pose", pose_json);
    cJSON_AddNumberToObject(root, "v", v);
    cJSON_AddNumberToObject(root, "r", (int)(r * 1000));
    cJSON_AddNumberToObject(root, "loop", loop);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

/***** ********************************START***************************************
 * 20211103修改: 增加对机械臂的关节示教,位置示教,姿态示教和示教停止
 * *********************************************************************************/
//发送控制机械臂关节示教的指令
int SetJointTeachCmd(int16_t teach_joint, std::string direction, int16_t v)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_joint_teach");
    cJSON_AddNumberToObject(root, "teach_joint", teach_joint);
    cJSON_AddStringToObject(root, "direction", direction.c_str());
    cJSON_AddNumberToObject(root, "v", v);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    //    mutex.lock();

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    //    ros::Duration(0.1).sleep();
    //    mutex.unlock();

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送控制机械臂位置示教的指令
int SetPosTeachCmd(std::string teach_type, std::string direction, int16_t v)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_pos_teach");
    cJSON_AddStringToObject(root, "teach_type", teach_type.c_str());
    cJSON_AddStringToObject(root, "direction", direction.c_str());
    cJSON_AddNumberToObject(root, "v", v);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    //    mutex.lock();

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    //    ros::Duration(0.1).sleep();
    //    mutex.unlock();

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送控制机械臂姿态示教的指令
int SetOrtTeachCmd(std::string teach_type, std::string direction, int16_t v)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_ort_teach");
    cJSON_AddStringToObject(root, "teach_type", teach_type.c_str());
    cJSON_AddStringToObject(root, "direction", direction.c_str());
    cJSON_AddNumberToObject(root, "v", v);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    //    mutex.lock();

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    //    ros::Duration(0.1).sleep();
    //    mutex.unlock();

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送控制机械臂停止示教的指令
int SetStopTeachCmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_stop_teach");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    //    mutex.lock();

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    //    ros::Duration(0.1).sleep();
    //    mutex.unlock();

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}
/***** ********************************END****************************************/

/***** ********************************START***************************************
 * 20220628修改: 增加对机械臂切换工具坐标系、切换工作坐标系、查询机械臂状态
 * *********************************************************************************/

//发送控制机械臂执行MoveJ_P的指令
int Movej_p_Cmd(POSE pose, byte v)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    //位置数据，0.001mm
    cJSON_AddNumberToObject(array, "test", (int)(pose.px * 1000000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.py * 1000000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.pz * 1000000));
    //姿态数据，0.001rad
    cJSON_AddNumberToObject(array, "test", (int)(pose.rx * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.ry * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.rz * 1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movej_p");
    cJSON_AddItemToObject(root, "pose", array);
    cJSON_AddNumberToObject(root, "v", v);
    cJSON_AddNumberToObject(root, "r", (int)(r * 1000));

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送控制机械臂执行切换当前工具坐标系的指令
int ChangeToolName_Cmd(std::string toolname)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    int r = 0;

    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_change_tool_frame");
    cJSON_AddStringToObject(root, "tool_name", toolname.c_str());

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    //    mutex.lock();

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    //    ros::Duration(0.1).sleep();
    //    mutex.unlock();

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送控制机械臂执行切换当前工作坐标系的指令
int ChangeWorkFrame_Cmd(std::string WorkFrame_name)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    int r = 0;

    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_change_work_frame");
    cJSON_AddStringToObject(root, "frame_name", WorkFrame_name.c_str());

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    //    mutex.lock();

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    //    ros::Duration(0.1).sleep();
    //    mutex.unlock();

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送控制机械臂获取当前机械臂状态的指令
int GetCurrentArmState_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    int r = 0;

    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_current_arm_state");

    // if(command == "get_current_arm_state")
    // {
    //     //加入字符串对象
    //     cJSON_AddStringToObject(root, "command", "get_current_arm_state");
    // }
    // else
    // {
    //     ROS_INFO("The command to get arm current state is wrong!");
    //     return 1;
    // }

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//发送控制机械臂获取当前机械臂状态的指令
int GetArmState_Cmd(std::string command)
{
    return GetCurrentArmState_Cmd();
}
/*************************************END****************************************/

/***** ********************************START***************************************
 * 20220808修改: 增加对机械臂的复合拖动示教、力位混合控制、结束力位混合控制
 * *********************************************************************************/
//开始复合模式拖动示教
int Start_Multi_Drag_Teach_Cmd(byte mode)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "start_multi_drag_teach");
    cJSON_AddNumberToObject(root, "mode", mode);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Stop_Drag_Teach_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "stop_drag_teach");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);

    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//力位混合控制
int Set_Force_Position_Cmd(byte mode, byte sensor, int N, byte load)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_force_position");
    cJSON_AddNumberToObject(root, "sensor", sensor);
    cJSON_AddNumberToObject(root, "mode", mode);
    cJSON_AddNumberToObject(root, "N", N);
    cJSON_AddNumberToObject(root, "load", load);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//结束力位混合控制
int Stop_Force_Postion_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "stop_force_position");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Start_Force_Position_Move_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "Start_Force_Position_Move");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Stop_Force_Position_Move_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "Stop_Force_Position_Move");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Force_Position_Move_Pose_Cmd(byte mode, byte sensor, byte dir, int force, POSE pose)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    //位置数据，0.001mm
    cJSON_AddNumberToObject(array, "test", (int)(pose.px * 1000000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.py * 1000000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.pz * 1000000));
    //姿态数据，0.001rad
    cJSON_AddNumberToObject(array, "test", (int)(pose.rx * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.ry * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(pose.rz * 1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "Force_Position_Move");
    cJSON_AddItemToObject(root, "pose", array);
    cJSON_AddNumberToObject(root, "sensor", sensor);
    cJSON_AddNumberToObject(root, "mode", mode);
    cJSON_AddNumberToObject(root, "dir", dir);
    cJSON_AddNumberToObject(root, "force", force);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    send_mutex.lock();
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    send_mutex.unlock();
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Force_Position_Move_Jiont_Cmd(byte mode, byte sensor, byte dir, int force, float *joint)
{
    cJSON *root, *array;
    char *data;
    char buffer[300];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    cJSON_AddNumberToObject(array, "test", (int)(joint[0] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[1] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[2] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[3] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[4] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[5] * 1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "Force_Position_Move");
    cJSON_AddItemToObject(root, "joint", array);
    cJSON_AddNumberToObject(root, "sensor", sensor);
    cJSON_AddNumberToObject(root, "mode", mode);
    cJSON_AddNumberToObject(root, "dir", dir);
    cJSON_AddNumberToObject(root, "force", force);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    send_mutex.lock();
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    send_mutex.unlock();

    if (res < 0)
    {
        return 1;
    }
    return 0;
}
//
int GetSixForce_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_force_data");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int ClearForceData_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "clear_force_data");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int SetForceSensor_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_force_sensor");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int ManualSetForcePose_Cmd(int *joint)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    cJSON_AddNumberToObject(array, "test", joint[0]);
    cJSON_AddNumberToObject(array, "test", joint[1]);
    cJSON_AddNumberToObject(array, "test", joint[2]);
    cJSON_AddNumberToObject(array, "test", joint[3]);
    cJSON_AddNumberToObject(array, "test", joint[4]);
    cJSON_AddNumberToObject(array, "test", joint[5]);

    for (int i = 0; i < 6; i++)
    {
        ROS_INFO("joint[%d]:%d", i, joint[i]);
    }
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "manual_set_force_pose1");
    cJSON_AddItemToObject(root, "joint", array);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int StopSetForceSensor_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "stop_set_force_sensor");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}
/***** ********************************END****************************************/

//角度透传
int Movej_CANFD(float *joint)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    cJSON_AddNumberToObject(array, "test", (int)(joint[0] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[1] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[2] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[3] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[4] * 1000));
    cJSON_AddNumberToObject(array, "test", (int)(joint[5] * 1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movej_canfd");
    cJSON_AddItemToObject(root, "joint", array);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    send_mutex.lock();
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    send_mutex.unlock();
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }

    return 0;
}
//急停指令
int Move_Stop_Cmd(void)
{
    cJSON *root;
    char *data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_arm_stop");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

// Set_DO_State:设置数字IO输出
int Set_DO_State(byte num, bool state)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_DO_state");
    cJSON_AddNumberToObject(root, "IO_Num", num);
    if (state)
    {
        cJSON_AddNumberToObject(root, "state", 1);
    }
    else
    {
        cJSON_AddNumberToObject(root, "state", 0);
    }

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

// Set_AO_State:设置模拟IO输出
int Set_AO_State(byte num, float voltage)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_AO_state");
    cJSON_AddNumberToObject(root, "IO_Num", num);
    cJSON_AddNumberToObject(root, "voltage", (int)(voltage * 1000));

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//获取所有IO输入状态
int Get_IO_Input(void)
{
    cJSON *root;
    char *data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_IO_input");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

// get robot joint
int Get_Arm_Joint(void)
{
    cJSON *root;
    char *data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_joint_degree");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

// Set_Tool_DO_State:设置工具端数字IO输出
int Set_Tool_DO_State(byte num, bool state)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_tool_DO_state");
    cJSON_AddNumberToObject(root, "IO_Num", num);
    if (state)
    {
        cJSON_AddNumberToObject(root, "state", 1);
    }
    else
    {
        cJSON_AddNumberToObject(root, "state", 0);
    }

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

// Set_Tool_AO_State:设置工具端模拟IO输出
int Set_Tool_AO_State(float voltage)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_tool_AO_state");
    cJSON_AddNumberToObject(root, "voltage", (int)(voltage * 1000));

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}
//获取工具端所有IO输入状态
int Get_Tool_IO_Input(void)
{
    cJSON *root;
    char *data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_tool_IO_input");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}
//设置手爪松开
int Set_Gripper_Release(int speed)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_gripper_release");
    cJSON_AddNumberToObject(root, "speed", speed);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}
//设置手爪力控夹取
int Set_Gripper_Pick(int speed, int force)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_gripper_pick");
    cJSON_AddNumberToObject(root, "speed", speed);
    cJSON_AddNumberToObject(root, "force", force);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//设置手爪开口度
int Set_Gripper(int position)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_gripper_position");
    cJSON_AddNumberToObject(root, "position", position);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

// set joint Enable state
int Set_Joint_Enable(int num, bool state)
{
    cJSON *root, *array;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_joint_en_state");
    cJSON_AddNumberToObject(array, "test", num);
    cJSON_AddNumberToObject(array, "test", (int)(state));
    cJSON_AddItemToObject(root, "joint_en_state", array);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}

// clear joint err
int Clear_Joint_Err(int num)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_joint_clear_err");
    cJSON_AddNumberToObject(root, "joint_clear_err", num);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if (res < 0)
    {
        return 1;
    }
    return 0;
}
/***************************************************解码函数************************************************************/
// Parser Arm Joint
int Parser_Arm_Joint(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    int data[6];
    int i = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "joint");
    if (result != NULL && result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if (size == 6)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                data[i] = json_sub->valueint;
                RM_Joint.joint[i] = data[i];
                RM_Joint.joint[i] = RM_Joint.joint[i] / 1000;
            }

            result = cJSON_GetObjectItem(root, "arm_err");
            if (result != NULL && result->type == cJSON_Number)
            {
                arm_err = result->valueint;
                // ROS_INFO("******************arm_err: %d********************", arm_err);
            }
            cJSON_Delete(root);
            return 0;
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        return 1;
    }
}

// Parser Arm Pose
int Parser_Arm_Pose(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    float pose[6];
    int data[6];
    int i = 0;
    geometry_msgs::Quaternion q_msg;
    tf2::Quaternion q_tf;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "pose");
    if (result != NULL && result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if (size == 6)
        {
            for (i = 0; i < size; i++)
            {
                pose[i] = cJSON_GetArrayItem(result, i)->valueint;
            }
            arm_pose.position.x = pose[0] / 1000000;
            arm_pose.position.y = pose[1] / 1000000;
            arm_pose.position.z = pose[2] / 1000000;

            q_tf.setRPY(pose[3] / 1000, pose[4] / 1000, pose[5] / 1000);
            q_msg = tf2::toMsg(q_tf); // tf类型转换为msg类型

            arm_pose.orientation.x = q_msg.x;
            arm_pose.orientation.y = q_msg.y;
            arm_pose.orientation.z = q_msg.z;
            arm_pose.orientation.w = q_msg.w;

            result = cJSON_GetObjectItem(root, "arm_err");
            if (result != NULL && result->type == cJSON_Number)
            {
                arm_err = result->valueint;
                // ROS_INFO("******************arm_err: %d********************", arm_err);
            }

            result = cJSON_GetObjectItem(root, "joint");
            if (result != NULL && result->type == cJSON_Array)
            {
                size = cJSON_GetArraySize(result);
                if (size == 6)
                {
                    for (i = 0; i < size; i++)
                    {
                        json_sub = cJSON_GetArrayItem(result, i);
                        data[i] = json_sub->valueint;
                        RM_Joint.joint[i] = data[i];
                        RM_Joint.joint[i] = RM_Joint.joint[i] / 1000;
                    }
                    cJSON_Delete(root);
                    return 4;
                }
                else
                {
                    cJSON_Delete(root);
                    return 0;
                }
            }
            else
            {
                cJSON_Delete(root);
                return 0;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        return 1;
    }
}

//解析机械臂关节错误代码
int Parser_Get_Joint_Err_Flag(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    int data[6];
    int i = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "err_flag");
    if (result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if (size == 6)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                data[i] = json_sub->valueint;
                RM_Joint.err_flag[i] = (uint16_t)(data[i]);
            }
            cJSON_Delete(root);
            return 0;
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        return 1;
    }
}

//解析所有IO输入通道的状态
int Parser_IO_Input(char *msg)
{
    cJSON *root = NULL, *result, *json_sub, *json_state;
    root = cJSON_Parse(msg);
    int data[6];
    int i = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    //获取数字输入
    result = cJSON_GetObjectItem(root, "DI");
    if (result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        for (i = 0; i < size; i++)
        {
            json_sub = cJSON_GetArrayItem(result, i);
            RM_Joint.Arm_DI[i] = json_sub->valueint;
        }
    }
    else
    {
        return 1;
    }
    //获取模拟输入
    result = cJSON_GetObjectItem(root, "AI");
    if (result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        for (i = 0; i < size; i++)
        {
            json_sub = cJSON_GetArrayItem(result, i);
            RM_Joint.Arm_AI[i] = (float)(json_sub->valueint) / 1000;
        }
    }
    else
    {
        return 1;
    }

    cJSON_Delete(root);
    return 0;
}
//解析工具端所有IO输入通道的状态
int Parser_Tool_IO_Input(char *msg)
{
    cJSON *root = NULL, *result, *json_sub, *json_state;
    root = cJSON_Parse(msg);
    int data[6];
    int i = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    //获取数字输入
    result = cJSON_GetObjectItem(root, "DI");
    if (result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        for (i = 0; i < size; i++)
        {
            json_sub = cJSON_GetArrayItem(result, i);
            RM_Joint.Tool_DI[i] = json_sub->valueint;
        }
    }
    else
    {
        return 1;
    }
    //获取模拟输入
    result = cJSON_GetObjectItem(root, "AI");
    if (result->type == cJSON_Number)
    {
        RM_Joint.Tool_AI = (float)(result->valueint) / 1000;
        cJSON_Delete(root);
        return 0;
    }
    else
    {
        return 1;
    }

    cJSON_Delete(root);
    return 0;
}

int Parser_Plan_State(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "trajectory_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.plan_flag = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
}
//切换当前工具坐标系
int Parser_ChangeTool_State(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "change_tool_frame");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.changeTool_flag = result->valueint;
        ROS_INFO("RM_Joint.changeTool_flag: %d", RM_Joint.changeTool_flag);
        cJSON_Delete(root);
        return 0;
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
}
//切换当前工作坐标系
int Parser_ChangeWorkFrame_State(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "change_work_frame");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.ChangeWorkFrame_flag = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
}
//查询机械臂当前状态
int Parser_Arm_Current_State(char *msg)
{
    cJSON *root = NULL, *arm_state, *joint, *pose, *arm_err, *sys_err;
    root = cJSON_Parse(msg);
    int i = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    arm_state = cJSON_GetObjectItem(root, "arm_state");
    joint = cJSON_GetObjectItem(arm_state, "joint");
    pose = cJSON_GetObjectItem(arm_state, "pose");

    for (i = 0; i < 6; i++)
    {
        Arm_Current_State.joint[i] = cJSON_GetArrayItem(joint, i)->valueint;
        Arm_Current_State.Pose[i] = cJSON_GetArrayItem(pose, i)->valueint;
        // ROS_INFO("Arm_Current_State.joint[%d] : %d",i,Arm_Current_State.joint[i]);
        // ROS_INFO("Arm_Current_State.Pose[%d] : %d",i,Arm_Current_State.Pose[i]);
    }

    arm_err = cJSON_GetObjectItem(arm_state, "arm_err");
    sys_err = cJSON_GetObjectItem(arm_state, "sys_err");
    // ROS_INFO("Arm_Current_State.arm_err: %d", arm_err->valueint);

    if (startMulitiDragTeach)
    {
        Arm_Current_State.arm_err = 0;
    }
    else
    {
        Arm_Current_State.arm_err = arm_err->valueint;
    }
    Arm_Current_State.sys_err = sys_err->valueint;

    // ROS_INFO("Arm_Current_State.arm_err: %d", Arm_Current_State.arm_err);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 1;
    }
    else
    {
        cJSON_Delete(root);
        return 0;
    }
}

int Parser_Force_Position_State(char *msg)
{
    cJSON *root = NULL, *result, *json_sub, *force;
    root = cJSON_Parse(msg);
    int data[6];
    int i = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "joint");
    force = cJSON_GetObjectItem(root, "force");

    if (result == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    if (force == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    int size = cJSON_GetArraySize(result);
    if (size == 6)
    {
        for (i = 0; i < size; i++)
        {
            json_sub = cJSON_GetArrayItem(result, i);
            data[i] = json_sub->valueint;
            RM_Joint.joint[i] = data[i];
            RM_Joint.joint[i] = RM_Joint.joint[i] / 1000;
        }
    }
    else
    {
        cJSON_Delete(root);
        return 3;
    }

    result = cJSON_GetObjectItem(root, "arm_err");
    if (result != NULL && result->type == cJSON_Number)
    {
        arm_err = result->valueint;
    }

    RM_Joint.force = force->valueint / 10;

    return 0;
}

int Parser_Six_Force_Data(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    int data[6];
    int i = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "force_data");
    if (result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if (size == 6)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                data[i] = json_sub->valueint;
                RM_Joint.six_force[i] = data[i];
                RM_Joint.six_force[i] = RM_Joint.six_force[i] / 1000;
                // ROS_INFO("RM_Joint.six_force[%d]:%f",i, RM_Joint.six_force[i]);
            }
            cJSON_Delete(root);
            return 0;
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        return 1;
    }
}

int Parser_Start_Multi_Drag_Teach(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "set_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Stop_Drag_Teach(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "drag_teach");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Stop_Force_Position(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "stop_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Set_Force_Position(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "set_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Clear_force_Data(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "clear_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Set_force_Sensor(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "set_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Stop_Set_force_Sensor(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "stop_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Start_Force_Position_Move(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "set_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Stop_Force_Position_Move(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "set_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Force_Position_Move(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "set_state");
    if ((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        RM_Joint.state = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

// Parser Current Joint Current
int Parser_Current_Joint_Current(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    int i = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "joint_current");
    if (result != NULL && result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if (size == 6)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                RM_Joint.joint_current[i] = json_sub->valueint;
                RM_Joint.joint_current[i] = RM_Joint.joint_current[i] / 1000;
            }
            cJSON_Delete(root);
            return 0;
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        return 1;
    }
}

int Parser_Lift_State(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "height");
    if (result != NULL && result->type == cJSON_Number)
    {
        Lift_Current_State.height = result->valueint;

        result = cJSON_GetObjectItem(root, "current");
        if (result != NULL && result->type == cJSON_Number)
        {
            Lift_Current_State.current = result->valueint;
        }

        result = cJSON_GetObjectItem(root, "err_flag");
        if (result != NULL && result->type == cJSON_Number)
        {
            Lift_Current_State.err_flag = result->valueint;
        }

        result = cJSON_GetObjectItem(root, "mode");
        if (result != NULL && result->type == cJSON_Number)
        {
            Lift_Current_State.mode = result->valueint;
        }
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Msg(char *msg)
{
    cJSON *root = NULL, *json_state;
    root = cJSON_Parse(msg);
    // ROS_INFO("all msg data is:%s", msg);
    int res = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return -1;
    }

    json_state = cJSON_GetObjectItem(root, "joint");
    if (json_state != NULL)
    {
        json_state = cJSON_GetObjectItem(root, "state");
        // if(json_state == NULL)
        // {
        //     json_state = cJSON_GetObjectItem(root, "command");
        //     if(json_state == NULL)
        //     {
        //         res = Parser_Arm_Joint(msg);
        //         if(res == 0)
        //         {
        //             cJSON_Delete(root);
        //             return ARM_JOINT_STATE;
        //         }
        //         else
        //         {
        //             cJSON_Delete(root);
        //             return -2;
        //         }
        //     }
        // }
        if (json_state != NULL && json_state->type == cJSON_String && (!strcmp("joint_degree", json_state->valuestring) || !strcmp("joint_state", json_state->valuestring)))
        {
            // ROS_INFO("Parser_Arm_Joint");
            res = Parser_Arm_Joint(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return ARM_JOINT_STATE;
            }

            else
            {
                cJSON_Delete(root);
                return -2;
            }
        }
    }

    json_state = cJSON_GetObjectItem(root, "state");
    if (json_state != NULL)
    {
        if (json_state->type == cJSON_String && (!strcmp("pose_state", json_state->valuestring)))
        {
            // ROS_INFO("Parser_Arm_Pose");
            res = Parser_Arm_Pose(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return ARM_POSE_STATE;
            }
            else if (res == 4)
            {
                cJSON_Delete(root);
                return ARM_POSE_AND_JOINT_STATE;
            }
            else
            {
                cJSON_Delete(root);
                return -2;
            }
        }

        if (json_state->type == cJSON_String && !strcmp("Force_Position_State", json_state->valuestring))
        {
            res = Parser_Force_Position_State(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return FORCE_POSITION_STATE;
            }
            else
            {
                cJSON_Delete(root);
                return -3;
            }
        }

        if (json_state->type == cJSON_String && !strcmp("current_joint_current", json_state->valuestring))
        {
            res = Parser_Current_Joint_Current(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return ARM_CURRENT_JOINT_CURRENT;
            }
            else
            {
                cJSON_Delete(root);
                return -3;
            }
        }

        if (json_state->type == cJSON_String && !strcmp("lift_state", json_state->valuestring))
        {
            res = Parser_Lift_State(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return LIFT_CURRENT_STATE;
            }
            else
            {
                cJSON_Delete(root);
                return -3;
            }
        }

        // Test
        if (json_state->type == cJSON_String && !strcmp("total_work_frame", json_state->valuestring))
        {
            ROS_INFO("*****total_work_frame data is:%s", msg);
        }
    }

    json_state = cJSON_GetObjectItem(root, "err_flag");
    if (json_state != NULL)
    {
        res = Parser_Get_Joint_Err_Flag(msg);
        if (res == 0)
        {
            cJSON_Delete(root);
            return ARM_JOINT_ERR;
        }
        else
        {
            cJSON_Delete(root);
            return -3;
        }
    }

    json_state = cJSON_GetObjectItem(root, "AI");
    if (json_state != NULL)
    {
        // Arm IO Input
        if (json_state->type == cJSON_Array)
        {
            res = Parser_IO_Input(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return ARM_IO_INPUT;
            }
            else
            {
                cJSON_Delete(root);
                return -6;
            }
        }
        // Tool IO Input
        else
        {
            res = Parser_Tool_IO_Input(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return TOOL_IO_INPUT;
            }
            else
            {
                cJSON_Delete(root);
                return -4;
            }
        }
    }

    json_state = cJSON_GetObjectItem(root, "trajectory_state");
    if (json_state != NULL)
    {
        res = Parser_Plan_State(msg);
        if (res == 0)
        {
            cJSON_Delete(root);
            return PLAN_STATE_TYPE;
        }
        else
        {
            cJSON_Delete(root);
            return -5;
        }
    }

    json_state = cJSON_GetObjectItem(root, "change_tool_frame");
    if (json_state != NULL)
    {
        res = Parser_ChangeTool_State(msg);
        if (res == 0)
        {
            cJSON_Delete(root);
            return CHANGE_TOOL_NAME;
        }
        else
        {
            cJSON_Delete(root);
            return -3;
        }
    }

    json_state = cJSON_GetObjectItem(root, "change_work_frame");
    if (json_state != NULL)
    {
        res = Parser_ChangeWorkFrame_State(msg);
        if (res == 0)
        {
            cJSON_Delete(root);
            return CHANGE_WORK_FRAME;
        }
        else
        {
            cJSON_Delete(root);
            return -3;
        }
    }

    json_state = cJSON_GetObjectItem(root, "arm_state");
    if (json_state != NULL)
    {
        // ROS_INFO("all msg data is:%s", msg);
        res = Parser_Arm_Current_State(msg);
        if (res == 0)
        {
            cJSON_Delete(root);
            return ARM_CURRENT_STATE;
        }
        else
        {
            cJSON_Delete(root);
            return -3;
        }
    }

    // add new to judge the return of arm
    //  json_state = cJSON_GetObjectItem(root, "state");
    //  if(json_state != NULL)
    //  {
    //      // ROS_INFO("*****************************cJSON_GetObjectItem:  state");
    //      if(json_state->type == cJSON_String && !strcmp("Force_Position_State", json_state->valuestring))
    //      {
    //          res = Parser_Force_Position_State(msg);
    //          if(res == 0)
    //          {
    //              cJSON_Delete(root);
    //              return FORCE_POSITION_STATE;
    //          }
    //          else
    //          {
    //              cJSON_Delete(root);
    //              return -3;
    //          }
    //      }
    //  }
    //  ROS_INFO("*****************************cJSON_GetObjectItem:  command");

    json_state = cJSON_GetObjectItem(root, "command");
    if (json_state != NULL)
    {
        if (!strcmp("start_multi_drag_teach", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                res = Parser_Start_Multi_Drag_Teach(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return START_MULTI_DRAG_TEACH;
                }
                else
                {
                    cJSON_Delete(root);
                    return -7;
                }
            }
        }
        else if (!strcmp("stop_drag_teach", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "drag_teach");
            if (json_state != NULL)
            {
                res = Parser_Stop_Drag_Teach(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return STOP_DRAG_TEACH;
                }
                else
                {
                    cJSON_Delete(root);
                    return -7;
                }
            }
        }
        else if (!strcmp("set_force_postion", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                res = Parser_Set_Force_Position(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return SET_FORCE_POSITION;
                }
                else
                {
                    cJSON_Delete(root);
                    return -7;
                }
            }
        }
        else if (!strcmp("Start_Force_Position_Move", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                res = Parser_Start_Force_Position_Move(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return START_FORCE_POSITION_MOVE;
                }
                else
                {
                    cJSON_Delete(root);
                    return -7;
                }
            }
        }
        else if (!strcmp("Force_Position_Move", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                res = Parser_Force_Position_Move(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return FORCE_POSITION_MOVE;
                }
                else
                {
                    cJSON_Delete(root);
                    return -7;
                }
            }
        }
        else if (!strcmp("Stop_Force_Position_Move", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                res = Parser_Stop_Force_Position_Move(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return STOP_FORCE_POSITION_MOVE;
                }
                else
                {
                    cJSON_Delete(root);
                    return -7;
                }
            }
        }
        else if (!strcmp("clear_force_data", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "clear_state");
            if (json_state != NULL)
            {
                res = Parser_Clear_force_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return CLEAR_FORCE_DATA;
                }
                else
                {
                    cJSON_Delete(root);
                    return -7;
                }
            }
        }
        else if (!strcmp("set_force_sensor", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                res = Parser_Set_force_Sensor(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return FORCE_SENSOR_SET;
                }
                else
                {
                    cJSON_Delete(root);
                    return -7;
                }
            }
        }
        else if (!strcmp("stop_set_force_sensor", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "stop_state");
            if (json_state != NULL)
            {
                res = Parser_Stop_Set_force_Sensor(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return STOP_SET_FORCE_SENSOR;
                }
                else
                {
                    cJSON_Delete(root);
                    return -7;
                }
            }
        }
        else if (!strcmp("get_force_data", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "force_data");
            if (json_state != NULL)
            {
                res = Parser_Six_Force_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return GET_SIX_FORCE;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if (!strcmp("stop_force_postion", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "stop_state");
            if (json_state != NULL)
            {
                res = Parser_Stop_Force_Position(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return STOP_FORCE_POSTION;
                }
                else
                {
                    cJSON_Delete(root);
                    ROS_INFO("Parser stop_force_postion failed");
                    return -7;
                }
            }
        }
        else if (json_state->type == cJSON_String && !strcmp("set_gripper", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    set_gripper_result = true;
                    return SET_GRIPPER_STATE;
                }
                else if (json_state->type == cJSON_False)
                {
                    set_gripper_result = false;
                    return SET_GRIPPER_STATE;
                }
            }
        }
    }

    cJSON_Delete(root);
    // ROS_INFO("***************************parser msg return 0");
    return 0;
}

// Quater to Euler
POSE Quater_To_Euler(geometry_msgs::Pose target)
{
    POSE msg;

    msg.px = target.position.x;
    msg.py = target.position.y;
    msg.pz = target.position.z;

    msg.rx = atan2(2 * (target.orientation.w * target.orientation.x + target.orientation.y * target.orientation.z), (1 - 2 * (target.orientation.x * target.orientation.x + target.orientation.y * target.orientation.y)));
    msg.ry = asin(2 * (target.orientation.w * target.orientation.y - target.orientation.z * target.orientation.x));
    msg.rz = atan2(2 * (target.orientation.w * target.orientation.z + target.orientation.x * target.orientation.y), (1 - 2 * (target.orientation.y * target.orientation.y + target.orientation.z * target.orientation.z)));

    return msg;
}

//处理关节错误
int Info_Joint_Err(void)
{
    int i = 0;

    for (i = 0; i < 6; i++)
    {
        switch (RM_Joint.err_flag[i])
        {
        case ERR_MASK_OK:
            break;
        case ERR_MASK_OVER_CURRENT:
            ROS_ERROR("Joint %d over current err!\n", i + 1);
            break;
        case ERR_MASK_OVER_VOLTAGE:
            ROS_ERROR("Joint %d over voltage err!\n", i + 1);
            break;
        case ERR_MASK_UNDER_VOLTAGE:
            ROS_ERROR("Joint %d under voltage err!\n", i + 1);
            break;
        case ERR_MASK_OVER_TEMP:
            ROS_ERROR("Joint %d over temperature err!\n", i + 1);
            break;
        case ERR_MASK_HALL:
            ROS_ERROR("Joint %d Hall err!\n", i + 1);
            break;
        case ERR_MASK_ENC:
            ROS_ERROR("Joint %d encoder err!\n", i + 1);
            break;
        case ERR_MASK_POS_TRACK:
            ROS_ERROR("Joint %d position track err!\n", i + 1);
            break;
        case ERR_MASK_CUR_ON:
            ROS_ERROR("Joint %d cunrrent sensor err!\n", i + 1);
            break;
        case ERR_MASK_TEMP:
            ROS_ERROR("Joint %d temperature sensor err!\n", i + 1);
            break;
        case ERR_MASK_TAG_POS:
            ROS_ERROR("Joint %d target postion over limit err!\n", i + 1);
            break;
        case JOINT_CAN_LOSE_ERR:
            ROS_ERROR("Joint %d off-line!\n", i + 1);
            break;
        default:
            ROS_ERROR("Joint %d Multi Errors!\n", i + 1);
            break;
        }
    }

    return 0;
}

//系统错误输出
void Info_Arm_Err(void)
{
    switch (arm_err)
    {
    case ARM_OK:
        break;
    case ARM_ERR_JOINT_COMMUNICATE:
        ROS_ERROR("arm error: joint communicate error!\n");
        break;
    case ARM_ERR_TARGET_ANGLE_OVERRUN:
        ROS_ERROR("arm error:  target angle overrun error!\n");
        break;
    case ARM_ERR_UNREACHABLE:
        ROS_ERROR("arm error:  unreachable!\n");
        break;
    case ARM_ERR_KERNEL_COMMUNICATE:
        ROS_ERROR("arm error:  kernel communicate error!\n");
        break;
    case ARM_ERR_JOINT_BUS:
        ROS_ERROR("arm error:  joint communication bus error!\n");
        break;
    case ARM_ERR_PLAN_KERNEL:
        ROS_ERROR("arm error:  plan layer kernel error!\n");
        break;
    case ARM_ERR_JOINT_OVERSPEED:
        ROS_ERROR("arm error: joint overspeed!\n");
        break;
    case ARM_ERR_TIP_BOARD_CONNECT:
        ROS_ERROR("arm error: tip board can't connect!\n");
        break;
    case ARM_ERR_JOINT_LOCK:
        ROS_ERROR("arm error: joint locking!\n");
        break;
    case ARM_ERR_DRAG_TEACH_OVERSPEED:
        ROS_ERROR("arm error: drag teach overspeed!\n");
        break;
    case ARM_ERR_CRASH:
        ROS_ERROR("arm error: arm crash!\n");
        break;
    case ARM_ERR_NO_WCS:
        ROS_ERROR("arm error: no WCS!\n");
        break;
    case ARM_ERR_NO_TCS:
        ROS_ERROR("arm error: no TCS!\n");
        break;
    case ARM_ERR_JOINT_DISENABLE:
        ROS_ERROR("arm error: joint disenable error!\n");
        break;
    default:
        ROS_ERROR("arm error: unknow error!\n");
        break;
    }
    arm_err = 0;
}

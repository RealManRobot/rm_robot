#include <iostream>
#include <string.h>
#include <vector>
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
#include <signal.h>             // signal functions ctrl+c
#include<assert.h>              // UDP协议依赖库
#include <sstream>              // 获取机械臂版本所需库@HermanYe


#include <sys/ioctl.h>          // 设置非阻塞需要用到的头文件
#include <sys/time.h>
#include <sys/select.h>         //使用fd_set结构体时使用。
#include <fcntl.h>  

// 尝试解决中文乱码的问题加的 
#include <locale>

// messgae
#include <rm_msgs/Arm_Analog_Output.h>
#include <rm_msgs/Arm_Digital_Output.h>
#include <rm_msgs/Arm_IO_State.h>
#include <rm_msgs/Arm_Software_Version.h>
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
#include <rm_msgs/Hand_Status.h>
#include <rm_msgs/LiftState.h>
/********************************23.8.4*****************************************/
#include "std_msgs/UInt16.h"
#include <sensor_msgs/JointState.h>
#include <rm_msgs/Set_Realtime_Push.h>
/********************************24.12.16*****************************************/
#include <rm_msgs/Arm_Current_Status.h>
#include <rm_msgs/Joint_En_Flag.h>
#include <rm_msgs/Joint_Speed.h>
#include <rm_msgs/Joint_PoseEuler.h>
#include <rm_msgs/Joint_Temperature.h>
#include <rm_msgs/Joint_Voltage.h>
#include <rm_msgs/Lift_In_Position.h>
#include <rm_msgs/CartePosCustom.h>
#include <rm_msgs/JointPosCustom.h>
#include <rm_msgs/Force_Position_Move_Pose_Custom.h>
/***** ***************2025.01.08 添加对modbus的设置和协议适配***********************/
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt8.h"
#include <rm_msgs/RS485_Mode.h>
#include <rm_msgs/Set_Modbus_Mode.h>
#include <rm_msgs/Register_Data.h>
#include <rm_msgs/Read_Register.h>
#include <rm_msgs/Write_Register.h>
#include <rm_msgs/Err.h>
#include <rm_msgs/Rm_Plus_Base.h>
#include <rm_msgs/Rm_Plus_State.h>
/***** ***************2025.05.13 添加四代控制器适配***********************/
#include <rm_msgs/Moveloffset.h>
#include <rm_msgs/Softwarebuildinfo.h>
#include <rm_msgs/Trajectoryinfo.h>
#include <rm_msgs/Trajectorylist.h>
#include <rm_msgs/Gettrajectorylist.h>
#include <rm_msgs/Flowchartrunstate.h>
#include <rm_msgs/Jointversion.h>
#include <rm_msgs/Mastername.h>
#include <rm_msgs/Modbusreaddata.h>
#include <rm_msgs/Modbustcpmasterinfo.h>
#include <rm_msgs/Modbustcpmasterlist.h>
#include <rm_msgs/UpdateTCPmasterparam.h>
#include <rm_msgs/Get_TCP_Master_List_Param.h>
#include <rm_msgs/RS485params.h>

#include <rm_msgs/Read_TCPandRTU.h>

#include <rm_msgs/Write_TCPandRTU.h>

#include <rm_msgs/Read_ModbusRTU.h>
#include <rm_msgs/Write_ModbusRTU.h>

/***** ***************2025.11.03 ***********************/
#include <rm_msgs/Expand_In_Position.h>
#include <rm_msgs/Expand_Position.h>
#include <rm_msgs/Expand_Speed.h>
#include <rm_msgs/ExpandState.h>

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
// #define RAD_DEGREE 57.2958
#define RAD_DEGREE 57.295791433
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
#define ARM_ERR_OVERSPEED_LIMIT 0x1009      //超速度限制
#define ARM_ERR_ACCELERATION_LIMIT 0x100A   //超加速度限制
#define ARM_ERR_JOINT_LOCK 0x100B           //关节抱闸未打开
#define ARM_ERR_DRAG_TEACH_OVERSPEED 0x100C //拖动示教时超速
#define ARM_ERR_CRASH 0x100D                //机械臂发生碰撞
#define ARM_ERR_NO_WCS 0x100E               //无该工作坐标系
#define ARM_ERR_NO_TCS 0x100F               //无该工具坐标系
#define ARM_ERR_JOINT_DISENABLE 0x1010      //关节发生掉使能错误


#define DATA_SIZE 300000

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

typedef struct
{
    int16_t pos;    //扩展关节角度，单位度，精度 0.001°。
    int16_t current;   //当前升降驱动电流，单位：mA，精度：1mA。
    uint16_t err_flag; //驱动错误代码
    uint16_t en_flag; //扩展关节使能状态
    byte mode;         //当前升降状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动。
    uint16_t joint_id; //扩展关节ID
} EXPAND_STATE;
EXPAND_STATE Expand_Current_State;

//机械臂状态结构体，用于查询机械臂状态
typedef struct
{
    int32_t Pose[6];  //位姿
    int32_t joint[7]; //关节角度
    std::vector<uint16_t> error;  //机械臂报错
} ARM_STATE;
ARM_STATE Arm_Current_State;

// 定义一个结构体以保存手部数据
typedef struct {
    int hand_err;
    uint16_t hand_force[6];
    uint16_t hand_angle[6];
    uint16_t hand_state[6];
    uint16_t hand_pos[6];
} HandData;
HandData Udp_Hand_Data;

// 定义一个结构体以保存末端设备实时信息数据
// typedef struct {
//     int hand_err;
//     uint16_t hand_force[6];
//     uint16_t hand_angle[6];
//     uint16_t hand_state[6];
//     uint16_t hand_pos[6];
// } HandData;
// HandData Udp_Hand_Data;

// // 定义一个结构体以保存手部数据
// typedef struct {
//     int hand_err;
//     uint16_t hand_force[6];
//     uint16_t hand_angle[6];
//     uint16_t hand_state[6];
//     uint16_t hand_pos[6];
// } HandData;
// HandData Udp_Hand_Data;

//机械臂状态参数
typedef struct
{
    float    joint[7];                 //关节角度
    uint16_t err_flag[7];              //关节错误代码
    int8_t   Arm_DI[4];
    float    Arm_AI[4];
    int8_t   Tool_IO_Mode[2];
    int8_t   Tool_IO_State[2];
    bool     plan_flag;
    bool     changeTool_flag;          //切换工具坐标系
    bool     ChangeWorkFrame_flag;     //切换工作坐标系
    float    gripper_joint;            // gripper
    float    force;                    //当前力传感器原始数据0.001N或0.001Nm
    float    six_force[6];
    bool     state;                    //命令执行状态
    float    joint_current[6];
/**********************添加变量***********************/
    bool     en_flag[7];               //当前关节使能状态 ，1为上使能，0为掉使能
    float    joint_position[3];        //当前关节角度，精度0.001°
    float    joint_temperature[7];     //当前关节温度，精度0.001℃
    float    joint_voltage[7];         //当前关节电压，精度0.001V
    float    joint_speed[7];           //当前关节速度，精度0.02RPM
    float    joint_euler[3];           //欧拉角
    float    joint_quat[4];            //四元数
    float    zero_force[6];            //当前力传感器系统外受力数据0.001N或0.001Nm
    float    work_zero_force[6];       //当前工作坐标系下系统受到的外力数据
    float    tool_zero_force[6];       //当前该工具坐标系下系统受到的外力数据             
    float    joint_zero_force; 
    uint32_t mode_result;              //模式查询结果
    std::string   product_version;     //机械臂的型号信息
    std::string   plan_version;        //控制器版本信息
    std::string   arm_current_status;  //机械臂的当前状态
} JOINT_STATE;
JOINT_STATE RM_Joint;
JOINT_STATE Udp_RM_Joint;
/****************************************************************************************/
typedef struct
{
    bool joint_acc_;                        //
    bool tail_end_;                         //
    bool joint_speed_;                      //关节速度
    bool lift_state_;                       //升降关节
    bool expand_state_;                     //扩展关节
    bool arm_current_status_;               //机械臂当前状态
    bool aloha_state_;                      //aloha主臂状态
    bool hand_;                             //灵巧手状态显示
    bool rm_plus_state_;                    //末端设备实时信息
    bool rm_plus_base_;                     //末端设备实时信息
} custom_set;
/****************************************************************************************/
/******************************udp端口情况变量**23.8.9*Author kaola************************/
typedef struct
{
    uint16_t udp_cycle;               //udp协议周期情况，实际数值是5的倍数，如写1为1X5=5
    uint16_t udp_port;                //udp协议端口情况
    uint16_t udp_force_coordinate;    //反馈当前的六维力基准坐标系
    std::string udp_ip;   
    custom_set custom_set_data;  
} UDP_parameter;
UDP_parameter Udp_Setting;
/*******************************************节点启动参数***************************************/
int         arm_dof;                    //关节自由度
std::string Arm_IP_;                    //机械臂TCP_IP地址
std::string Udp_IP_;                    //机械臂UDP_IP地址
int Arm_Port_;                          //机械臂TCP端口地址
int Udp_Port_;                          //机械臂UDP端口地址
int Udp_cycle_;                         //机械臂UDP上报频率
int Udp_force_coordinate;               //机械臂六维力参考坐标系
bool canfd_follow = false;              //透传跟随方式
bool udp_hand_ = false;                 //灵巧手使能状态
bool udp_plus_state_ = false;           //末端设备实时信息
bool udp_plus_base_ = false;           //末端设备基础信息
int trajectory_mode_;                   //高跟随模式选择
int radio_;                             //平滑系数
bool is_4th_Gen_;                       //控制器版本选择
/*******************************udp赋值变量*23.8.4*Author kaola********************************/
bool realtime_arm_joint_state = true;
char udp_failed_time = 0;
uint16_t arm_err = 0;                //机械臂错误代码
uint16_t sys_err = 0;                //系统错误代码
std::vector<uint16_t> udp_err;     //机械臂\系统错误代码
bool set_gripper_result = false;
bool tcp_arm_joint_state = true;     //Tcp通信是否正常
/*********************************23.8.9添加变量 Author kaola********************************/
// 监控ctrl+c的标志位。
volatile sig_atomic_t ctrl_flag = 0;
// UDP读取成功标志位
bool connect_udp_flage = false;
// 六维力或一维力标志位
char force_sensor = 0;
/************************************udp数据变量 23.8.9添加变量 Author kaola********************************/
geometry_msgs::Pose udp_arm_pose;
rm_msgs::Six_Force Udp_Six_Force;
rm_msgs::Six_Force Udp_Six_Zero_Force;
rm_msgs::Gripper_Set arm_joint_error;
rm_msgs::Err udp_error;
std_msgs::UInt16 udp_sys_error;
std_msgs::UInt16 udp_arm_error;
std_msgs::UInt16 udp_coordinate;
rm_msgs::Manual_Set_Force_Pose udp_joint_error_code;
sensor_msgs::JointState udp_real_joint;
rm_msgs::Hand_Status udp_hand_status;
rm_msgs::Rm_Plus_Base udp_plus_base;
rm_msgs::Rm_Plus_State udp_plus_state;
/*******************************************************************************************************/

/*******************************************************************************************************/
rm_msgs::Arm_Current_Status udp_arm_current_status;
rm_msgs::Joint_Current udp_joint_current;
rm_msgs::Joint_En_Flag udp_joint_en_flag;
rm_msgs::Joint_Speed udp_joint_speed;
rm_msgs::Joint_Temperature udp_joint_temperature;
rm_msgs::Joint_Voltage udp_joint_voltage;
rm_msgs::Joint_PoseEuler udp_joint_poseEuler;

rm_msgs::Lift_In_Position lift_in_position;
rm_msgs::Expand_In_Position expand_in_position;
/*******************************************************************************************************/


/*********************************Modbus数据变量 ***************************************************************/
typedef struct{
    std_msgs::Bool state;
    rm_msgs::RS485_Mode get_controller_RS485_mode;
    rm_msgs::RS485_Mode get_tool_RS485_mode;

    rm_msgs::Register_Data read_coils;
    rm_msgs::Register_Data read_multiple_coils; //
    rm_msgs::Register_Data read_input_status;
    rm_msgs::Register_Data read_holding_registers;
    rm_msgs::Register_Data read_multiple_holding_registers; //
    rm_msgs::Register_Data read_input_registers;
    rm_msgs::Register_Data read_multiple_input_registers; //
}Modbus_Data;
Modbus_Data modbus_data;

/*******************************************************************************************************/
/*********************************四代控制器新增 ***************************************************************/
rm_msgs::Trajectorylist trajectory_list;
rm_msgs::Flowchartrunstate flowchart_runstate;
//rm_msgs::Arm_Softversion_v3 arm_software_info_v3;
//rm_msgs::Arm_Softversion_v4 arm_software_info_v4;
rm_msgs::Arm_Software_Version arm_version;
std_msgs::String tool_software_version;
rm_msgs::Jointversion joint_software_version;
rm_msgs::Modbustcpmasterinfo modbustcp_master_info;
rm_msgs::Modbustcpmasterlist modbustcp_master_list;
rm_msgs::RS485params get_modbus_mode;
rm_msgs::Register_Data read_modbus_data;
int Modbus_Read_Case;            //modbus读数据情况
int Modbus_Write_Case;           //modbus写数据情况
/*******************************************************************************************************/


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

ros::Subscriber sub_setHandPosture, sub_setHandSeq, sub_setHandAngle, sub_setHandSpeed, sub_setHandForce,sub_setHandFollowAngle, sub_setHandFollowPos;

// subscriber
ros::Subscriber MoveJ_Cmd, MoveJ_P_Cmd, MoveL_Cmd, MoveC_Cmd, JointPos_Cmd, Arm_DO_Cmd, Arm_AO_Cmd, Tool_DO_Cmd, Tool_AO_Cmd, Gripper_Cmd, Gripper_Set_Cmd, Emergency_Stop, Joint_En, System_En, IO_Update, Sub_ChangeToolName, Sub_ChangeWorkFrame, Sub_GetArmState, sub_getArmStateTimerSwitch, Sub_StartMultiDragTeach, Sub_StopDragTeach, Sub_SetForcePosition, Sub_StopForcePosition, Sub_StartForcePositionMove, Sub_StopForcePositionMove, Sub_ForcePositionMovePose, Sub_ForcePositionMoveJiont, Sub_ToGetSixForce, Sub_ClearForceData, Sub_SetForceSensor, Sub_ManualSetForcePose, Sub_StopSetForceSensor, Sub_GetArmJoint;
ros::Subscriber sub_setGripperPickOn;
ros::Subscriber MoveJ_Fd_Custom_Cmd, MoveP_Fd_Custom_Cmd, Sub_ForcePositionMovePoseCustom;
/*************************获取一维力数据*23.9.1添加变量 Author kaola**********/
ros::Subscriber Sub_ToGetOneForce;
// publisher
ros::Publisher Joint_State, Arm_IO_State, Tool_IO_State, Plan_State, ChangeTool_Name, ChangeWorkFrame_Name, ArmCurrentState, pub_Force_Position_State, pub_StartMultiDragTeach_Result, pub_StopDragTeach_Result, pub_SetForcePosition_Result, pub_StopForcePosition_Result, pub_ClearForceData_Result, pub_ForceSensorSet_Result, pub_StopSetForceSensor_Result, pub_StartForcePositionMove_Result, pub_StopForcePositionMove_Result, pub_Force_Position_Move_Result, pub_PoseState, pub_HandStatus, pub_RmPlusState, pub_RmPlusBase, pub_JointCurrent, pub_JointEnFlag, pub_JointSpeed, pub_JointTemperature, pub_JointVoltage, pub_ArmCurrentStatus, pub_PoseEuler, pub_LiftInPosition, pub_ExpandInPosition;
ros::Publisher pub_currentJointCurrent;
ros::Publisher pub_armCurrentState;
ros::Publisher pub_liftState;
ros::Publisher pub_setGripperResult;
/***************************modbus协议适配 2025.01.08 Author Fan*********************************/
ros::Subscriber sub_setRS485, sub_getControllerRS485Mode, sub_getToolRS485Mode, sub_setModbusMode, sub_closeModbusMode, sub_setModbustcpMode, sub_closeModbustcpMode;
ros::Subscriber sub_readCoils, sub_readMultipleCoils, sub_writeSingleCoil, sub_writeCoils, sub_readInputStatus, sub_readHoldingRegisters, sub_writeSingleRegister, sub_writeRegisters, sub_readMultipleHoldingRegisters, sub_readInputRegisters, sub_readMultipleInputRegisters;

ros::Publisher pub_getControllerRS485Mode_result, pub_getToolRS485Mode_result, pub_setModbusMode_result, pub_closeModbusMode_result, pub_setModbustcpMode_result, pub_closeModbustcpMode_result;
ros::Publisher pub_readCoils_result, pub_readMultipleCoils_result, pub_writeSingleCoil_result, pub_writeCoils_result, pub_readInputStatus_result, pub_readHoldingRegisters_result, pub_writeSingleRegister_result, pub_writeRegisters_result,  pub_readMultipleHoldingRegisters_result, pub_readInputRegisters_result, pub_readMultipleInputRegisters_result;
/*************************当前机械臂报错、系统报错、关节报错话题*23.8.9添加变量 Author kaola**********/
ros::Publisher pub_ArmError, pub_SysError, pub_JointErrorCode, pub_UdpError;
/*************************当前力传感器系统外受力数据0.001N或0.001Nm*23.8.9添加变量 Author kaola**********/
/*************************原始力数据、系统受到的外力、工作坐标系下、工具坐标系下****************************/
ros::Publisher pub_GetSixForce, pub_SixZeroForce, pub_Work_Zero_Force, pub_Tool_Zero_Force;     
/**************************************Udp发布的外力传感器数据*****************************************/
ros::Publisher pub_UdpSixForce, pub_UdpSixZeroForce;
/*************************高低跟随选择的话题变量*23.8.9添加变量 Author kaola********/
// ros::Subscriber Set_Movej_Canfd_Follow, Set_Movep_Canfd_Follow;
/*************************高低跟随选择的话题变量*23.8.11添加变量 Author kaola********/
// ros::Subscriber Get_Movej_Canfd_Follow, Get_Movep_Canfd_Follow;
/***************************udp参数设置查询**23.8.9添加变量 Author kaola*************/
ros::Subscriber Set_Realtime_Push, Get_Realtime_Push;
ros::Publisher Set_Realtime_Push_Result, Get_Realtime_Push_Result;
/**********************************示教返回话题****************************/
ros::Publisher pub_setJointTeachResult;     //关节
ros::Publisher pub_setPosTeachResult;       //位置
ros::Publisher pub_setOrtTeachResult;       //姿态
ros::Publisher pub_setStopResult;           //停止
ros::Publisher pub_setLiftSpeedResult;      //升降速度开环控制
/*********************************灵巧手返回话题*******************************/
ros::Publisher pub_setHandPostureResult;    //手势
ros::Publisher pub_setHandSeqResult;        //序列
ros::Publisher pub_setHandHAngleResult;     //由度角度
ros::Publisher pub_set_HandSpeedResult;     //速度
ros::Publisher pub_setHandForceResult;      //阈值
ros::Publisher pub_setHandFollowAngleResult;//角度跟随
ros::Publisher pub_setHandFollowPosResult;  //姿势跟随
/*****************************设置机械臂电源使能返回话题**************************/
ros::Publisher pub_setArmPowerResult;
/*****************************设置工具端电源输出返回话题**************************/
ros::Publisher pub_setToolVoltageResult;
/*****************************设置工具端数字IO输出状态**************************/
ros::Publisher pub_setToolDOStateResult;
ros::Publisher pub_setDOStateResult;
ros::Publisher pub_setAOStateResult;
ros::Publisher pub_setArmStopResult;
/**********************************清除关节报错***********************************/
ros::Publisher pub_Joint_Clear_Err_Result;
/**********************************使能、失能关节*********************************/
ros::Publisher pub_Joint_En_State_Result;
/**********************************清除系统报错***********************************/
ros::Publisher pub_System_En_State_Result;
/*******************************发布当前的受力基准坐标系***************************/
ros::Publisher pub_Udp_Coordinate;
/**********************************控制器获取版本信息*************************/
ros::Subscriber Get_Arm_Software_Version_Cmd;
ros::Publisher Get_Arm_Software_Version_Result;



/***********************************四代控制器新增***********************************/
/***********************************获取关节软件版本信息**********************************/
ros::Subscriber Get_Joint_Software_Version_Cmd;
ros::Publisher Get_Joint_Software_Version_Result;
/***********************************获取末端接口板软件版本信息**********************************/
ros::Subscriber Get_Tool_Software_Version_Cmd;
ros::Publisher Get_Tool_Software_Version_Result;
/***********************************设置机械臂急停状态***********************************/
ros::Subscriber Set_Arm_Emergency_Stop_cmd;
ros::Publisher Set_Arm_Emergency_Stop_Result;
/***********************************查询流程图编程状态***********************************/
ros::Subscriber Get_Trajectory_File_List_cmd;
ros::Publisher Get_Trajectory_File_List_Result;
/***********************************开始运行指定轨迹***********************************/
ros::Subscriber Set_Run_Trajectory_cmd;
ros::Publisher Set_Run_Trajectory_Result;
/***********************************删除指定轨迹***********************************/
ros::Subscriber Delete_Trajectory_File_cmd;
ros::Publisher Delete_Trajectory_File_Result;
/***********************************保存轨迹到控制机器***********************************/
ros::Subscriber Save_Trajectory_File_cmd;
ros::Publisher Save_Trajectory_File_Result;
/***********************************查询流程图编程状态***********************************/
ros::Subscriber Get_Flowchart_Program_Run_State_cmd;
ros::Publisher Get_Flowchart_Program_Run_State_Result;
/***********************************笛卡尔空间直线偏移运动***********************************/
ros::Subscriber Movel_Offset_cmd;
ros::Publisher Movel_Offset_Result;
/***********************************新增Modbus TCP主站***********************************/
ros::Subscriber Add_Modbus_Tcp_Master_cmd;
ros::Publisher Add_Modbus_Tcp_Master_Result;
/***********************************修改Modbus TCP主站***********************************/
ros::Subscriber Update_Modbus_Tcp_Master_cmd;
ros::Publisher Update_Modbus_Tcp_Master_Result;
/***********************************删除Modbus TCP主站***********************************/
ros::Subscriber Delete_Modbus_Tcp_Master_cmd;
ros::Publisher Delete_Modbus_Tcp_Master_Result;
/***********************************查询Modbus TCP主站***********************************/
ros::Subscriber Get_Modbus_Tcp_Master_cmd;
ros::Publisher Get_Modbus_Tcp_Master_Result;
/***********************************查询TCP主站列表***********************************/
ros::Subscriber Get_Modbus_Tcp_Master_List_cmd;
ros::Publisher Get_Modbus_Tcp_Master_List_Result;
/***********************************设置控制器RS485模式(四代控制器支持)***********************************/
ros::Subscriber Set_Controller_Rs485_Mode_cmd;
ros::Publisher Set_Controller_Rs485_Mode_Result;
/***********************************查询控制器RS485模式(四代控制器支持)***********************************/
ros::Subscriber Get_Controller_Rs485_Mode_V4_cmd;
ros::Publisher Get_Controller_Rs485_Mode_V4_Result;
/***********************************设置工具端RS485模式(四代控制器支持)***********************************/
ros::Subscriber Set_Tool_Rs485_Mode_cmd;
ros::Publisher Set_Tool_Rs485_Mode_Result;
/***********************************查询工具端RS485模式(四代控制器支持)***********************************/
ros::Subscriber Get_Tool_Rs485_Mode_V4_cmd;
ros::Publisher Get_Tool_Rs485_Mode_V4_Result;

/***********************************Modbus TCP协议读线圈和查询结果***********************************/
ros::Subscriber Read_Modbus_Coils_cmd;
ros::Publisher Read_Modbus_Coils_Result;
/***********************************Modbus TCP协议写线圈和查询结果***********************************/
ros::Subscriber Write_Modbus_Coils_cmd;
ros::Publisher Write_Modbus_Coils_Result;
/***********************************Modbus TCP协议读离散量输入和查询结果***********************************/
ros::Subscriber Read_Modbus_Input_Status_cmd;
ros::Publisher Read_Modbus_Input_Status_Result;
/***********************************Modbus TCP协议读保持寄存器和查询结果***********************************/
ros::Subscriber Read_Modbus_Holding_Registers_cmd;
ros::Publisher Read_Modbus_Holding_Registers_Result;
/***********************************Modbus TCP协议写保持寄存器和查询结果***********************************/
ros::Subscriber Write_Modbus_Registers_cmd;
ros::Publisher Write_Modbus_Registers_Result;
/***********************************Modbus TCP协议读输入寄存器和查询结果***********************************/
ros::Subscriber Read_Modbus_Input_Registers_cmd;
ros::Publisher Read_Modbus_Input_Registers_Result;


/***********************************设置末端生态协议模式***********************************/
ros::Subscriber sub_setRmPlusMode;
ros::Publisher pub_setRmPlusModeResult;
/**********************************查询末端生态协议模式************************************/
ros::Subscriber sub_getRmPlusMode;
ros::Publisher pub_getRmPlusModeResult;
/***********************************设置触觉传感器模式***********************************/
ros::Subscriber sub_setRmPlusTouch;
ros::Publisher pub_setRmPlusTouchResult;
/**********************************查询触觉传感器模式************************************/
ros::Subscriber sub_getRmPlusTouch;
ros::Publisher pub_getRmPlusTouchResult;

/**********************************V2.6.0************************************/
/**********************************轨迹暂停************************************/
ros::Subscriber sub_setArmPause_Cmd;
ros::Publisher pub_setArmPause_Result;
/**********************************暂停后恢复功能************************************/
ros::Subscriber sub_setArmContinue_Cmd;
ros::Publisher pub_setArmContinue_Result;
/**********************************扩展关节状态获取************************************/
ros::Subscriber sub_getExpandState_Cmd;
ros::Publisher pub_getExpandState_Result;
/**********************************扩展关节速度环控制************************************/
ros::Subscriber sub_setExpandSpeed_Cmd;
ros::Publisher pub_setExpandSpeed_Result;
/**********************************扩展关节位置环控制************************************/
ros::Subscriber sub_setExpandPos_Cmd;
ros::Publisher pub_setExpandPos_Result;
// std::mutex mutex;

// timer
ros::Timer State_Timer;
//虚假的心跳
ros::Timer Fake_Socket_Heart;
// UDP定时器
ros::Timer Udp_State_Timer;
int timer_cnt = 0;

bool startMulitiDragTeach = false;

// Update:2023-7-25 @HermanYe
// Get controller version
uint16_t CONTROLLER_VERSION = 0;

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
#define STOP_FORCE_POSITION 0x13
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

// Update:2023-7-25 @HermanYe
// Get controller version
#define CTRL_VERSION 0x20
// set UDP Parameter
#define SET_REALTIME_PUSH 0X21
#define GET_REALTIME_PUSH 0X22
#define SET_JOINT_TEACH   0X23
#define SET_POS_TEACH     0X24
#define SET_ORT_TEACH     0X25
#define SET_STOP_TEACH    0X26
#define SET_LIFT_SPEED    0X27
/***************灵巧手*******************/
#define SET_HAND_POSTURE  0X28
#define SET_HAND_SEQ      0X29
#define SET_HAND_ANGLE    0X2A
#define SET_HAND_SPEED    0X2B
#define SET_HAND_FORCE    0X2C
/****************设置机械臂电源********************/
#define SET_ARM_POWER     0X2D
/*******************末端电压*********************/
#define SET_TOOL_VOLTAGE    0X2E
#define SET_TOOL_DO_STATE   0X2F
#define SET_DO_STATE        0X30
#define SET_AO_STATE        0X31
#define SET_ARM_STOP        0X32
#define SET_JOINT_CLEAR_ERR 0X33
#define GET_ONE_FORCE       0X34
#define SET_JOINT_EN_STATE  0x35
#define SET_SYSTEM_EN_STATE 0X36
#define HAND_FOLLOW_ANGLE   0X37
#define HAND_FOLLOW_POS     0X38
/*******************升降机构*********************/
#define LIFT_IN_POSITION    0X39
/*******************Modbus**********************/
#define GET_CONTROLLER_RS485_MODE 0X3A
#define GET_TOOL_RS485_MODE 0X3B
#define SET_MODBUS_MODE 0X3C
#define CLOSE_MODBUS_MODE 0X3D
#define SET_MODBUSTCP_MODE 0X3E
#define CLOSE_MODBUSTCP_MODE 0X3F

#define READ_COILS 0X40
#define READ_MULTIPLE_COILS 0X41
#define WRITE_SINGLE_COIL 0X42
#define WRITE_COILS 0X43
#define READ_INPUT_STATUS 0X44
#define READ_HOLDING_REGISTERS 0X45
#define WRITE_SINGLE_REGISTER 0X46
#define WRITE_REGISTERS 0X47
#define READ_MULTIPLE_HOLDING_REGISTERS 0X48
#define READ_INPUT_REGISTERS 0X49
#define READ_MULTIPLE_INPUT_REGISTERS 0X4A
#define SET_RM_PLUS_MODE 0X4B
#define GET_RM_PLUS_MODE 0X4C
#define SET_RM_PLUS_TOUCH 0X4D
#define GET_RM_PLUS_TOUCH 0X4E

/*******************四代控制器新增**********************/
#define SET_ARM_EMERGENCY_STOP 0X4F
#define GET_TRAJECTORY_FILE_LIST 0X51
#define SET_RUN_TRAJECTORY 0X52
#define DELETE_TRAJECTORY_FILE 0X53
#define SAVE_TRAJECTORY_FILE 0X54
#define GET_FLOWCHART_PROGRAM_RUN_STATE 0X55
#define MOVEL_OFFSET 0X56
#define ARM_VERSION 0X57
#define JOINT_VERSION 0X58
#define TOOL_VERSION 0X59
#define ADD_MODBUS_TCP_MASTER 0X5A
#define UPDATE_MODBUS_TCP_MASTER 0X5B
#define DELETE_MODBUS_TCP_MASTER 0X5C
#define GET_MODBUS_TCP_MASTER 0X5D
#define GET_MODBUS_TCP_MASTER_LIST 0X5E
#define SET_CONTROLLER_RS485_MODE 0X5F
#define GET_CONTROLLER_RS485_MODE_V4 0X61
#define SET_TOOL_RS485_MODE 0X62
#define GET_TOOL_RS485_MODE_V4 0X63
#define MODBUS_READ 0X64
#define MODBUS_WRITE 0X65

/*******************V2.6.0 **********************/
#define SET_ARM_CONTINUE 0X66
#define EXPAND_CURRENT_STATE 0X67
#define SET_EXPAND_SPEED 0X68
#define EXPAND_IN_POSITION 0X69
#define SET_ARM_PAUSE 0X6A

float min_interval = 0.02;              //透传周期,单位:秒
float udp_min_interval = 0.005;          //机械臂状态发布,单位:秒

int Arm_Socket;                         //机械臂TCp网络通信套接字
int Udp_Sockfd;                         //机械臂UDP网络通信套接字
int Arm_connect;                        //机械臂TCP连接状态
int Arm_udp_connect;                    //机械臂UDP连接状态

// const char *Arm_IP = "192.168.1.18"; //机械臂IP地址

std::mutex send_mutex;

bool ros_shutdown_flage = 0;            //ros使用ctrl+c退出时的标志变量，由于在回调函数时不好操作，所以使用这种方式退出
volatile int connect_status=1;          //连接状态变量，实时更新，在heart_callback函数中
volatile int last_connect_status=1;     //上一connect_status状态变量，防止重复执行相关操作多加的一个变量，主要在主程序下方，关闭线程时使用。

/**************************函数声明***************************/
int Info_Joint_Err(void);
void Info_Arm_Err(void);


// Unicode转义解码函数
void decode_unicode_escape(char* str) {
    char* p = str;
    char* q = str;
    while (*p) {
        if (*p == '\\' && *(p+1) == 'u') {
            // 1.提取四位十六进制值
            unsigned int code;
            sscanf(p+2, "%4x", &code);
            
            // 2.UTF-8编码转换
            if (code <= 0x7F) {
                *q++ = code;
            } else if (code <= 0x7FF) {
                *q++ = 0xC0 | (code >> 6);
                *q++ = 0x80 | (code & 0x3F);
            } else {
                *q++ = 0xE0 | (code >> 12);
                *q++ = 0x80 | ((code >> 6) & 0x3F);
                *q++ = 0x80 | (code & 0x3F);
            }
            p += 6;
        } else {
            *q++ = *p++;
        }
    }
    *q = '\0';
}


// send 套壳
ssize_t package_send(int Arm_Socket, const void *buffer, size_t buffer_size, int flage)
{
    send_mutex.lock();
    ssize_t res = send(Arm_Socket, buffer, buffer_size, flage);
    ros::Duration(min_interval).sleep();
    send_mutex.unlock();
    return res;
}

//连接机械臂网络   
int Arm_Socket_Start(void)
{
    // close(Arm_Socket);
    Arm_Socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (Arm_Socket <= 0)
    {
        return 2;
    }

    struct sockaddr_in serAddr;
    struct timeval tm;
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(Arm_Port_);
    serAddr.sin_addr.s_addr = inet_addr((char*)Arm_IP_.c_str());
    int flag,old_flag;
    flag |= O_NONBLOCK;
    // 设置为非阻塞模式
    old_flag = flag = fcntl(Arm_Socket, F_SETFL, O_NONBLOCK );
    // 查看连接状态
    Arm_connect = connect(Arm_Socket, (struct sockaddr *)&serAddr, sizeof(serAddr));
    // ROS_INFO("Arm_connect=%d\n",Arm_connect);
    if (Arm_connect != 0)
    {
        if(errno != EINPROGRESS) //connect返回错误。
		{
            close(Arm_Socket);
			ROS_ERROR("********************Connect faile check your connect!**************\n");
            return 3;
		}
        else
        {
            //1.定时依赖于头文件#include <sys/time.h>
            struct timeval tm;  
            //2s
			tm.tv_sec = 2;      
			tm.tv_usec = 0;

			fd_set wset;

            /* ;将指定的文件描述符集清空，在对文件描述符集合进行设置前，必须对其进行初始化，
            如果不清空，由于在系统分配内存空间后，通常并不作清空处理，所以结果是不可知的 */
			FD_ZERO(&wset);

            // 使用方式：
            /*FD_SET(10,&set);将set的第十位置一，也就是set.__fds_bits[0]为1024！
            FD_SET(32,&set);将set的第32位置一，也就是set.__fds_bits[1]为1！*/
            // 经过观察基本上Arm_Socket都为10所以这里处理完后就是在bit[0]的第十位写一也就是数值1024。
			FD_SET(Arm_Socket,&wset); 
            /*
            函数原型：int select(int nfds, fd_set *readset, fd_set *writeset,fd_set* exceptset, struct tim *timeout);
            函数功能：测试指定的fd可读？可写？有异常条件待处理？
            */
			int res = select(Arm_Socket+1, NULL, &wset, NULL, &tm);
            if(res <= 0)
			{
				ROS_ERROR("********************Connect faile check your connect!**************\n");
				close(Arm_Socket);
				return 4;
			}
            /*判断Arm_Socket是否在wset中*/
            if(FD_ISSET(Arm_Socket,&wset))
			{
				// ROS_INFO("test \n");
				int err = -1;
				socklen_t len = sizeof(int);
				/*
                *函数原型：int getsockopt(int socket, int level, int option_name,void *restrict option_value, socklen_t *restrict option_len);
                *返回值：    成功：0    失败：-1
                */
				if(getsockopt(Arm_Socket, SOL_SOCKET, SO_ERROR, &err, &len ) < 0) //两种错误处理方式
				{
					ROS_INFO("errno :%d %s\n",errno, strerror(errno));
					close(Arm_Socket);
					return 5;
				}
 
				if(err)
				{
					ROS_ERROR("********************Connect faile check your connect!**************\n");
					errno = err;
					close(Arm_Socket);
					return 6;
				}
			}

        }

    }
    fcntl(Arm_Socket, F_SETFL, old_flag); //最后恢复sock的阻塞属性。

    return 0;
}

//关闭机械臂网络
void Udp_Socket_Close()
{
    shutdown(Udp_Sockfd,SHUT_RDWR);
    close(Udp_Sockfd);
}

//连接机械臂UDP网络   
int Udp_Socket_Start(void)
{
    // UDP通信说明
    Udp_Sockfd = socket(AF_INET,SOCK_DGRAM,0);
    if (Udp_Sockfd < 0)
    {
        close(Udp_Sockfd);
        return 2; 
    }

    struct sockaddr_in saddr;
    memset(&saddr,0,sizeof(saddr));
    saddr.sin_family = AF_INET;
    // saddr.sin_addr.s_addr = inet_addr(Udp_IP);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(Udp_Port_);


    struct timeval timeout;
    timeout.tv_sec = 3;  // 超时时间为3秒
    timeout.tv_usec = 0;

    if (bind(Udp_Sockfd, (struct sockaddr*) & saddr, sizeof(saddr)) < 0) 
    {
        ROS_ERROR("errno :%d %s\n",errno, strerror(errno));
        Udp_Socket_Close();
        return 1;
    }

    setsockopt(Udp_Sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ROS_INFO("UDP Connect Success!!!");
    return 0;
}

static void my_handler(int sig)  // can be called asynchronously
{ 

  ctrl_flag = 1; // set flag

}

int SocketConnected(void)
{
    if(CONTROLLER_VERSION == 1)
    {
        //ROS_INFO("SocketConnected is in %d",CONTROLLER_VERSION);
        if(Arm_Socket <= 0)
        {
            close(Arm_Socket);
            return -1;
        }
        int err = -1;
        socklen_t len = sizeof(int);
           
        if(getsockopt(Arm_Socket, SOL_SOCKET, SO_ERROR, &err, &len ) < 0)
        {
            ROS_INFO("errno :%d %s\n",errno, strerror(errno));
            close(Arm_Socket);
            return 0;
        }
        if(err)
        {
            ROS_INFO("connect faile\n");
            errno = err;
            close(Arm_Socket);
            return 0;
        }
        return 1;
    }
    else
    {
        if(realtime_arm_joint_state == false)
        {
            ROS_ERROR("Connect IS failed");
            return 0;
        }
        else 
        return 1;
    }
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

// test
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
int SetHandPostureCmd(uint16_t posture_num, bool block)
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
    cJSON_AddBoolToObject(root, "block", block);

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
int SetHandSeqCmd(uint16_t seq_num, bool block)
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
    cJSON_AddBoolToObject(root, "block", block);

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
int SetHandAngle(int16_t *hand_angle,bool block)
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
    cJSON_AddBoolToObject(root, "block", block);


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

int SetHandFollowAngle(int16_t *hand_angle)
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
    cJSON_AddStringToObject(root, "command", "hand_follow_angle");
    cJSON_AddItemToObject(root, "hand_angle", array);

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

int SetHandFollowPos(int16_t *hand_angle)
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
    cJSON_AddStringToObject(root, "command", "hand_follow_pos");
    cJSON_AddItemToObject(root, "hand_pos", array);

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

    cJSON_Delete(root);
    free(data);

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

int Movep_CANFD(POSE pose, bool follow, uint8_t trajectory_mode, uint8_t radio)
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
    if(follow == false)
    {
        cJSON_AddFalseToObject(root, "follow");
    }
    else if(follow == true)
    {
        cJSON_AddTrueToObject(root, "follow");
    }
    cJSON_AddNumberToObject(root, "trajectory_mode", trajectory_mode);
    cJSON_AddNumberToObject(root, "radio", radio);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    //ROS_INFO("movep_canfd cammand: %s", buffer);
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

/****************************************Udp的配置函数************************************/
int Udp_Set_Realtime_Push(uint16_t cycle, uint16_t port, uint16_t force_coordinate, std::string ip, custom_set custom_data)
{
    cJSON *root;
    cJSON *custom_set = NULL;
    char *data;
    char buffer[400];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    custom_set = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_realtime_push");
    cJSON_AddNumberToObject(root, "cycle", cycle);

    cJSON_AddTrueToObject(root, "enable");

    cJSON_AddNumberToObject(root, "port", port);
    if(force_sensor != 12)
    {
        cJSON_AddNumberToObject(root, "force_coordinate", force_coordinate);
    }
    cJSON_AddStringToObject(root, "ip", ip.c_str());
    
    if(custom_data.tail_end_ == true)
        cJSON_AddTrueToObject(custom_set, "tail_end");
    else
        cJSON_AddFalseToObject(custom_set, "tail_end");
    if(custom_data.lift_state_ == true)
        cJSON_AddTrueToObject(custom_set, "lift_state");
    else
        cJSON_AddFalseToObject(custom_set, "lift_state");
    if(custom_data.joint_speed_ == true)
        cJSON_AddTrueToObject(custom_set, "joint_speed");
    else
        cJSON_AddFalseToObject(custom_set, "joint_speed");
    if(custom_data.joint_acc_ == true)
        cJSON_AddTrueToObject(custom_set, "joint_acc");
    else
        cJSON_AddFalseToObject(custom_set, "joint_acc");
    if(custom_data.hand_ == true)
        cJSON_AddTrueToObject(custom_set, "hand");
    else
        cJSON_AddFalseToObject(custom_set, "hand");
    if(custom_data.rm_plus_state_ == true)
        cJSON_AddTrueToObject(custom_set, "rm_plus_state");
    else
        cJSON_AddFalseToObject(custom_set, "rm_plus_state");
    if(custom_data.rm_plus_base_ == true)
        cJSON_AddTrueToObject(custom_set, "rm_plus_base");
    else
        cJSON_AddFalseToObject(custom_set, "rm_plus_base");
    if(custom_data.expand_state_ == true)
        cJSON_AddTrueToObject(custom_set, "expand_state");
    else
        cJSON_AddFalseToObject(custom_set, "expand_state");
    if(custom_data.arm_current_status_ == true)
        cJSON_AddTrueToObject(custom_set, "arm_current_status");
    else
        cJSON_AddFalseToObject(custom_set, "arm_current_status");
    if(custom_data.aloha_state_ == true)
        cJSON_AddTrueToObject(custom_set, "aloha_state");
    else
        cJSON_AddFalseToObject(custom_set, "aloha_state");
    cJSON_AddItemToObject(root, "custom", custom_set);
    
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
    Udp_Setting.udp_port = port;
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
    ROS_INFO("set_lift_height cammand: %s", buffer);
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
int Movej_Cmd(float *joint, byte v, uint8_t trajectory_connect)
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
    if(arm_dof == 7)
    {
        cJSON_AddNumberToObject(array, "test", (int)(joint[6] * 1000));
    }
    
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movej");
    cJSON_AddItemToObject(root, "joint", array);
    cJSON_AddNumberToObject(root, "v", v);
    cJSON_AddNumberToObject(root, "r", (int)(r * 1000));
    cJSON_AddNumberToObject(root, "trajectory_connect", trajectory_connect);

    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    ROS_INFO("movej command: %s", buffer);
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
int Movel_Cmd(POSE pose, byte v, u_int8_t trajectory_connect)
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
    cJSON_AddNumberToObject(root, "trajectory_connect", trajectory_connect);

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
int Movec_Cmd(POSE pose_via, POSE pose_to, uint16_t v, uint16_t loop, u_int8_t trajectory_connect)
{
    cJSON *root, *array1, *array2, *pose_json;
    char *data;
    char buffer[400];
    int res;
    int r = 0;
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
    cJSON_AddNumberToObject(root, "r", r);
    cJSON_AddNumberToObject(root, "loop", loop);
    cJSON_AddNumberToObject(root, "trajectory_connect", trajectory_connect);

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
int Movej_p_Cmd(POSE pose, byte v, uint8_t trajectory_connect)
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
    cJSON_AddNumberToObject(root, "trajectory_connect", trajectory_connect);

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
// old version
// int Start_Multi_Drag_Teach_Cmd(byte mode)
// {
//     cJSON *root;
//     char *data;
//     char buffer[200];
//     int res;
//     //创建根节点对象
//     root = cJSON_CreateObject();

//     //加入字符串对象
//     cJSON_AddStringToObject(root, "command", "start_multi_drag_teach");
//     cJSON_AddNumberToObject(root, "mode", mode);

//     data = cJSON_Print(root);

//     sprintf(buffer, "%s\r\n", data);
//     // res = send(Arm_Socket, buffer, strlen(buffer), 0);
//     res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
//     cJSON_Delete(root);
//     free(data);

//     if (res < 0)
//     {
//         return 1;
//     }
//     return 0;
// }
// new version

int Start_Multi_Drag_Teach_Cmd(int *free_axes,int frame, int singular_wall)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "start_multi_drag_teach");
    //数组加入数据
    for(int i =0;i<6;i++)
        cJSON_AddNumberToObject(array, "free_axes", free_axes[i]);
    cJSON_AddItemToObject(root, "free_axes", array);
    cJSON_AddNumberToObject(root, "frame", frame);


    cJSON_AddNumberToObject(root, "singular_wall", singular_wall);

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
int Set_Force_Position_Cmd(byte mode, byte sensor, int N, byte direction)
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
    cJSON_AddNumberToObject(root, "direction", direction);                 //direction：力控方向；0-沿X轴；1-沿Y轴；2-沿Z轴；
    cJSON_AddNumberToObject(root, "N", N);

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
int Stop_Force_Position_Cmd()
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

int Force_Position_Move_Pose_Cmd(byte mode, byte sensor, byte dir, int force, bool follow, uint8_t trajectory_mode, uint8_t radio, POSE pose)
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
    if(follow == false)
    {
        cJSON_AddFalseToObject(root, "follow");
    }
    else if(follow == true)
    {
        cJSON_AddTrueToObject(root, "follow");
    }
    cJSON_AddNumberToObject(root, "trajectory_mode", trajectory_mode);
    cJSON_AddNumberToObject(root, "radio", radio);

    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    //ROS_INFO("Force_Position_Move command: %s", buffer);
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
    if(arm_dof == 7)
    {
        cJSON_AddNumberToObject(array, "test", (int)(joint[6] * 1000));
    }

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "Force_Position_Move");
    cJSON_AddItemToObject(root, "joint", array);
    cJSON_AddNumberToObject(root, "sensor", sensor);
    cJSON_AddNumberToObject(root, "mode", mode);
    cJSON_AddNumberToObject(root, "dir", dir);
    cJSON_AddNumberToObject(root, "force", force);
    if(canfd_follow == false)
    {
        cJSON_AddFalseToObject(root, "follow");
    }
    else if(canfd_follow == true)
    {
        cJSON_AddTrueToObject(root, "follow");
    }

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

int GetOneForce_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_Fz");

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

int ManualSetForcePose_Cmd(std::string pose, long long *joint)
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

    // for (int i = 0; i < 6; i++)
    // {
    //     ROS_INFO("joint[%d]:%lld", i, joint[i]);
    // }
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", pose.c_str());//"manual_set_force_pose1"
    cJSON_AddItemToObject(root, "joint", array);

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
/**************************************END****************************************/

int clearsystemerr_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "clear_system_err");

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
/**************************************END****************************************/


/**********************************查询Udp的参数配置*********************************/
int GetRealtimePush_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_realtime_push");

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
//modbus模式配置
int Set_RS485_Cmd(uint32_t baudrate)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_RS485");
    cJSON_AddNumberToObject(root, "baudrate", baudrate);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    ROS_INFO("set_RS485 cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;

}
int Get_Controller_RS485_Mode_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_controller_RS485_mode");
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("get_controller_RS485_mode cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Get_Tool_RS485_Mode_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_tool_RS485_mode");
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("get_tool_RS485_mode cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Set_Modbus_Mode_Cmd(uint8_t port, uint32_t baudrate, uint32_t timeout)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_modbus_mode");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "baudrate", baudrate);
    cJSON_AddNumberToObject(root, "timeout", timeout);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("set_modbus_mode cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Close_Modbus_Mode_Cmd(uint8_t port)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "close_modbus_mode");
    cJSON_AddNumberToObject(root, "port", port);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("close_modbus_mode cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Set_Modbustcp_Mode_Cmd(std::string ip, uint32_t port, uint32_t timeout)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_modbustcp_mode");
    cJSON_AddStringToObject(root, "ip", ip.c_str());
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "timeout", timeout);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("set_modbustcp_mode cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Close_Modbustcp_Mode_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "close_modbustcp_mode");
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("close_modbustcp_mode cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
//Modbus协议指令
int Read_Coils_Cmd(uint8_t port, uint32_t address, uint32_t  num, uint32_t device)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "read_coils");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("read_coils cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Read_Multiple_Coils_Cmd(uint8_t port, uint32_t address, uint32_t  num, uint32_t device)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "read_multiple_coils");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("read_multiple_coils cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Write_Single_Coil_Cmd(uint8_t port, uint32_t address, uint16_t data_content, uint32_t device)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "write_single_coil");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "data", data_content);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("write_single_coil cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Write_Coils_Cmd(uint8_t port, uint32_t address, uint32_t num, std::vector<uint16_t> data_content, uint32_t device)
{
    cJSON *root,*array;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "write_coils");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    //数组加入数据
    for(int i =0;i<data_content.size();i++)
        cJSON_AddNumberToObject(array, "data", data_content[i]);
    cJSON_AddItemToObject(root, "data", array);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("write_coils cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Read_Input_Status_Cmd(uint8_t port, uint32_t address, uint32_t  num, uint32_t device)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "read_input_status");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("read_input_status cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Read_Holding_Registers_Cmd(uint8_t port, uint32_t address, uint32_t device)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "read_holding_registers");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("read_holding_registers cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Write_Single_Register_Cmd(uint8_t port, uint32_t address, uint16_t data_content, uint32_t device)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "write_single_register");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "data", data_content);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("write_single_register cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Write_Registers_Cmd(uint8_t port, uint32_t address, uint32_t num, std::vector<uint16_t> data_content, uint32_t device)
{
    cJSON *root,*array;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "write_registers");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    for(int i =0;i<data_content.size();i++)
        cJSON_AddNumberToObject(array, "data", data_content[i]);
    cJSON_AddItemToObject(root, "data", array);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("write_registers cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Read_Multiple_Holding_Registers_Cmd(uint8_t port, uint32_t address, uint32_t  num, uint32_t device)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "read_multiple_holding_registers");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("read_multiple_holding_registers cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Read_Input_Registers_Cmd(uint8_t port, uint32_t address, uint32_t device)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "read_input_registers");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("read_input_registers cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Read_Multiple_Input_Registers_Cmd(uint8_t port, uint32_t address, uint32_t  num, uint32_t device)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "read_multiple_input_registers");
    cJSON_AddNumberToObject(root, "port", port);
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    cJSON_AddNumberToObject(root, "device", device);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("read_multiple_input_registers cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Set_Rm_Plus_Mode_Cmd(uint32_t mode)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_rm_plus_mode");
    cJSON_AddNumberToObject(root, "mode", mode);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("set_rm_plus_mode cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Get_Rm_Plus_Mode_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_rm_plus_mode");
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("get_rm_plus_mode cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Set_Rm_Plus_Touch_Cmd(uint32_t mode)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_rm_plus_touch");
    cJSON_AddNumberToObject(root, "mode", mode);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("set_rm_plus_touch cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Get_Rm_Plus_Touch_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_rm_plus_touch");
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("get_rm_plus_touch cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Set_Rm_Arm_Continue_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_arm_continue");
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("get_rm_plus_touch cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Set_Rm_Arm_Pause_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_arm_pause");
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("get_rm_plus_touch cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Get_Expand_State_Cmd()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "expand_get_state");
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("get_rm_plus_touch cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Set_Expand_Speed_Cmd(int speed)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "expand_set_speed");
    cJSON_AddNumberToObject(root, "speed", speed);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("get_rm_plus_touch cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}
int Set_Expand_Pos_Cmd(int pos, int speed)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "expand_set_pos");
    cJSON_AddNumberToObject(root, "pos", pos);
    cJSON_AddNumberToObject(root, "speed", speed);
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("get_rm_plus_touch cmmand: %s",buffer);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}

//角度透传
int Movej_CANFD(float *joint,float expand,bool follow, uint8_t trajectory_mode, uint8_t radio)
{
    cJSON *root, *array;
    char *data;
    char buffer[200];
    int res,i;
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
    if(arm_dof == 7)
    {
        cJSON_AddNumberToObject(array, "test", (int)(joint[6] * 1000));
    }

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movej_canfd");
    cJSON_AddItemToObject(root, "joint", array);
    if(follow == false)
    {
        cJSON_AddFalseToObject(root, "follow");
    }
    else if(follow == true)
    {
        cJSON_AddTrueToObject(root, "follow");
    }
    cJSON_AddNumberToObject(root, "trajectory_mode", trajectory_mode);
    cJSON_AddNumberToObject(root, "radio", radio);

    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // ROS_INFO("movej_canfg send command: %s", buffer);
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
    /*  
        首先调用cJSON_ CreateObject ()函数，创建一个JSON对象，
        之后便可向这个对象中添加string或int等内容的数据项了。
        使用该函数会通过malloc()函数在内存中开辟一个空间，使用完成需要手动释放。
    */
    root = cJSON_CreateObject();
    /*
        将上一步生成的数据项与其键值（"firstName"）一起添加到root对象中。
        这里面有一个键值概念,键
    */
    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_joint_degree");
    /*
        将cJSON对象的内容解析为字符串，并展示出来。
    */
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // res = send(Arm_Socket, buffer, strlen(buffer), 0);
    // 调用套接字发送buffer中的数据内容
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    //  通过cJSON_Delete()，释放cJSON_CreateObject ()分配出来的内存空间。
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
    if(CONTROLLER_VERSION == 2)
    {
        cJSON_AddStringToObject(root, "command", "get_tool_IO_state");
    }
    // else if(CONTROLLER_VERSION == 1)
    // {
    //     cJSON_AddStringToObject(root, "command", "get_tool_IO_input");
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

//设置手爪chixu力控夹取
int Set_Gripper_Pick_on(int speed, int force)
{
    cJSON *root;
    char *data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_gripper_pick_on");
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
    int data[7];
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
        if (size == arm_dof)
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
    int data[7];
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
                if (size == arm_dof)
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
    int data[7];
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
        if (size == arm_dof)
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
    if(CONTROLLER_VERSION == 1)
    {
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
    if(CONTROLLER_VERSION == 2)
    {
        //获取数字输入
        result = cJSON_GetObjectItem(root, "IO_Mode");
        if (result->type == cJSON_Array)
        {
            int size = cJSON_GetArraySize(result);
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                RM_Joint.Tool_IO_Mode[i] = json_sub->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        result = cJSON_GetObjectItem(root, "IO_State");
        if (result->type == cJSON_Array)
        {
            int size = cJSON_GetArraySize(result);
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                RM_Joint.Tool_IO_State[i] = json_sub->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
    }

    cJSON_Delete(root);
    return 0;
}

int Parser_Lift_InPosition(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    result = cJSON_GetObjectItem(root, "device");
    if(result->type == cJSON_Number)
    {
        lift_in_position.device = result->valueint;
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    result = cJSON_GetObjectItem(root, "state");
    if(result->type == cJSON_String)
    {
        lift_in_position.state = result->valuestring;
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    result = cJSON_GetObjectItem(root, "trajectory_connect");
    if(result->type == cJSON_Number)
    {
        lift_in_position.trajectory_connect = result->valueint;
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    result = cJSON_GetObjectItem(root, "trajectory_state");
    if((result->type == cJSON_True) || (result->type == cJSON_False))
    {
        lift_in_position.trajectory_state = result->valueint;
    }
    else
    {
        cJSON_Delete(root);
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

// Update:2023-7-25 @HermanYe
// Get controller version

// Update:2025-05-13 @Poppy
// 四代控制器新增
int Get_Arm_Software_Version()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "get_arm_software_info");
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
int Get_Joint_Software_Version()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "get_joint_software_version");
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
int Get_Tool_Software_Version()
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    int r = 0;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "get_tool_software_version");
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
int Parse_Set_Arm_Emergency_Stop(bool stop_state){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "set_arm_emergency_stop");
    cJSON_AddNumberToObject(root, "emergency_stop", stop_state);
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

int Get_Trajectory_File_List(int page_num, int page_size, const char *vague_search){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "get_trajectory_file_list");
    cJSON_AddNumberToObject(root, "page_num", page_num);
    cJSON_AddNumberToObject(root, "page_size", page_size);
    cJSON_AddStringToObject(root, "vague_search", vague_search);
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
int Set_Run_Trajectory(std::string trajectory_name){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "set_run_trajectory_file");
    cJSON_AddStringToObject(root, "name", trajectory_name.c_str());
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
int Delete_Trajectory_File(std::string trajectory_name){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "delete_trajectory_file");
    cJSON_AddStringToObject(root, "name", trajectory_name.c_str());
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

int Save_Trajectory_File(std::string trajectory_name){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "save_trajectory_file");
    cJSON_AddStringToObject(root, "name", trajectory_name.c_str());
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;/*  */
    }
    return 0;
}

int Get_Flowchart_Program_Run_State(){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "get_flowchart_program_run_state");
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

int Movel_Offset(geometry_msgs::Pose offset, int v, int r, int trajectory_connect, int frame_type, int block)
{
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "movel_offset");
    cJSON* offset_arr = cJSON_CreateArray();
    cJSON_AddItemToArray(offset_arr, cJSON_CreateNumber(offset.position.x));
    cJSON_AddItemToArray(offset_arr, cJSON_CreateNumber(offset.position.y));
    cJSON_AddItemToArray(offset_arr, cJSON_CreateNumber(offset.position.z));
    cJSON_AddItemToArray(offset_arr, cJSON_CreateNumber(offset.orientation.x));
    cJSON_AddItemToArray(offset_arr, cJSON_CreateNumber(offset.orientation.y));
    cJSON_AddItemToArray(offset_arr, cJSON_CreateNumber(offset.orientation.z));
    cJSON_AddItemToArray(offset_arr, cJSON_CreateNumber(offset.orientation.w));
    cJSON_AddItemToObject(root, "offset", offset_arr);
    cJSON_AddNumberToObject(root, "v", v);
    cJSON_AddNumberToObject(root, "r", r);
    cJSON_AddNumberToObject(root, "trajectory_connect", trajectory_connect);
    cJSON_AddNumberToObject(root, "frame_type", frame_type);
    cJSON_AddNumberToObject(root, "block", block);
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
// ---------------------Modbus TCP 主站相关----------------------------------------
int Add_Modbus_Tcp_Master(std::string master_name,std::string ip,int port){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "add_modbus_tcp_master");
    cJSON_AddStringToObject(root, "master_name", master_name.c_str());
    cJSON_AddStringToObject(root, "ip", ip.c_str());
    cJSON_AddNumberToObject(root, "port", port);
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
int Update_Modbus_Tcp_Master(std::string master_name,std::string new_name, std::string ip,int port){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "update_modbus_tcp_master");
    cJSON_AddStringToObject(root, "master_name", master_name.c_str());
    cJSON_AddStringToObject(root, "new_name", new_name.c_str());
    cJSON_AddStringToObject(root, "ip", ip.c_str());
    cJSON_AddNumberToObject(root, "port", port);
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
int Delete_Modbus_Tcp_Master(std::string master_name){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "delete_modbus_tcp_master");
    cJSON_AddStringToObject(root, "master_name", master_name.c_str());
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
int Get_Modbus_Tcp_Master(std::string master_name){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "given_modbus_tcp_master");
    cJSON_AddStringToObject(root, "master_name", master_name.c_str());
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
int Get_Modbus_Tcp_Master_List(int page_num, int page_size, std::string vague_search){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "get_modbus_tcp_master_list");
    cJSON_AddNumberToObject(root, "page_num", page_num);
    cJSON_AddNumberToObject(root, "page_size", page_size);
    cJSON_AddStringToObject(root, "vague_search", vague_search.c_str());
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

int Set_Controller_Rs485_Mode(int mode, int baudrate){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "set_controller_rs485_mode");
    cJSON_AddNumberToObject(root, "mode", mode);
    cJSON_AddNumberToObject(root, "baudrate", baudrate);
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
int Get_Controller_Rs485_Mode_V4(){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "get_controller_rs485_mode");
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
int Set_Tool_Rs485_Mode(int mode, int baudrate){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "set_tool_rs485_mode");
    cJSON_AddNumberToObject(root, "mode", mode);
    cJSON_AddNumberToObject(root, "baudrate", baudrate);
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
int Get_Tool_Rs485_Mode_V4(){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "get_tool_rs485_mode");
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

//************************************************2025.6.17************************************************** */
int Read_Modbus_Tcp_Coils(int address,int num,std::string ip,int port,std::string master_name){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "read_modbus_tcp_coils");
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    // 优先使用IP和Port
    if (!ip.empty() && port > 0) {
        cJSON_AddStringToObject(root, "ip", ip.c_str());
        cJSON_AddNumberToObject(root, "port", port);
    }
    // 否则使用Master Name
    else if (!master_name.empty()) {
        cJSON_AddStringToObject(root, "master_name", master_name.c_str());
    }
    // 如果两者都没有，返回错误
    else {
        cJSON_Delete(root);
        return 1;
    }
    data = cJSON_Print(root);
    //printf("%s\n", data);
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

int Write_Modbus_Tcp_Coils(int address,std::vector<int> tcp_data,std::string ip,int port,std::string master_name){
    cJSON *root,*data_array;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "write_modbus_tcp_coils");
    cJSON_AddNumberToObject(root, "address", address);
    data_array = cJSON_CreateArray();
    for (int val : tcp_data) {
        cJSON_AddItemToArray(data_array, cJSON_CreateNumber(val));
    }
    cJSON_AddItemToObject(root, "data", data_array);
    // 优先使用IP和Port
    if (!ip.empty() && port > 0) {
        cJSON_AddStringToObject(root, "ip", ip.c_str());
        cJSON_AddNumberToObject(root, "port", port);
    }
    // 否则使用Master Name
    else if (!master_name.empty()) {
        cJSON_AddStringToObject(root, "master_name", master_name.c_str());
    }
    // 如果两者都没有，返回错误
    else {
        cJSON_Delete(root);
        return 1;
    }
    data = cJSON_Print(root);
    sprintf(buffer, "%s\r\n", data);
    // std::cout << data << std::endl;
    res = package_send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);
    if (res < 0)
    {
        return 1;
    }
    return 0;
}

int Read_Modbus_Tcp_Input_Status(int address, int num,std::string ip,int port,std::string master_name){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "read_modbus_tcp_input_status");
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    // 优先使用IP和Port
    if (!ip.empty() && port > 0) {
        cJSON_AddStringToObject(root, "ip", ip.c_str());
        cJSON_AddNumberToObject(root, "port", port);
    }
    // 否则使用Master Name
    else if (!master_name.empty()) {
        cJSON_AddStringToObject(root, "master_name", master_name.c_str());
    }
    // 如果两者都没有，返回错误
    else {
        cJSON_Delete(root);
        return 1;
    }
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

int Read_Modbus_TCP_Holding_Registers(int address, int num,std::string ip,int port,std::string master_name){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "read_modbus_tcp_holding_registers");
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    // 优先使用IP和Port
    if (!ip.empty() && port > 0) {
        cJSON_AddStringToObject(root, "ip", ip.c_str());
        cJSON_AddNumberToObject(root, "port", port);
    }
    // 否则使用Master Name
    else if (!master_name.empty()) {
        cJSON_AddStringToObject(root, "master_name", master_name.c_str());
    }
    // 如果两者都没有，返回错误
    else {
        cJSON_Delete(root);
        return 1;
    }
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

int Write_Modbus_Tcp_Registers(int address,std::vector<int> tcp_data,std::string ip,int port,std::string master_name){
    cJSON *root,*data_array;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "write_modbus_tcp_registers");
    cJSON_AddNumberToObject(root, "address", address);
    data_array = cJSON_CreateArray();
    for (int val : tcp_data) {
        cJSON_AddItemToArray(data_array, cJSON_CreateNumber(val));
    }
    cJSON_AddItemToObject(root, "data", data_array);
    // 优先使用IP和Port
    if (!ip.empty() && port > 0) {
        cJSON_AddStringToObject(root, "ip", ip.c_str());
        cJSON_AddNumberToObject(root, "port", port);
    }
    // 否则使用Master Name
    else if (!master_name.empty()) {
        cJSON_AddStringToObject(root, "master_name", master_name.c_str());
    }
    // 如果两者都没有，返回错误
    else {
        cJSON_Delete(root);
        return 1;
    }
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

int Read_Modbus_Tcp_Input_Registers(int address, int num,std::string ip,int port,std::string master_name){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "read_modbus_tcp_input_registers");
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "num", num);
    // 优先使用IP和Port
    if (!ip.empty() && port > 0) {
        cJSON_AddStringToObject(root, "ip", ip.c_str());
        cJSON_AddNumberToObject(root, "port", port);
    }
    // 否则使用Master Name
    else if (!master_name.empty()) {
        cJSON_AddStringToObject(root, "master_name", master_name.c_str());
    }
    // 如果两者都没有，返回错误
    else {
        cJSON_Delete(root);
        return 1;
    }
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

// ----------------------------------------------------------------------------
int Read_Modbus_Rtu_Coils(int address,int device,int num,int type){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "read_modbus_rtu_coils");
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "device", device);
    cJSON_AddNumberToObject(root, "num", num);
    cJSON_AddNumberToObject(root, "type", type);
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

int Write_Modbus_Rtu_Coils(int address,std::vector<int> rtu_data,int type,int device){
    cJSON *root,*data_array;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "write_modbus_rtu_coils");
    cJSON_AddNumberToObject(root, "address", address);
    data_array = cJSON_CreateArray();
    for (int val : rtu_data) {
        cJSON_AddItemToArray(data_array, cJSON_CreateNumber(val));
    }
    cJSON_AddItemToObject(root, "data", data_array);
    cJSON_AddNumberToObject(root, "type", type);
    cJSON_AddNumberToObject(root, "device", device);
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

int Read_Modbus_Rtu_Input_Status(int address,int device,int num,int type){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "read_modbus_rtu_input_status");
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "device", device);
    cJSON_AddNumberToObject(root, "num", num);
    cJSON_AddNumberToObject(root, "type", type);
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
int Read_Modbus_Rtu_Holding_Registers(int address,int device,int num,int type){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "read_modbus_rtu_holding_registers");
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "device", device);
    cJSON_AddNumberToObject(root, "num", num);
    cJSON_AddNumberToObject(root, "type", type);
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
int Write_Modbus_Rtu_Registers(int address,std::vector<int> rtu_data,int type,int device){
    cJSON *root,*data_array;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "write_modbus_rtu_registers");
    cJSON_AddNumberToObject(root, "address", address);
    data_array = cJSON_CreateArray();
    for (int val : rtu_data) {
        cJSON_AddItemToArray(data_array, cJSON_CreateNumber(val));
    }
    cJSON_AddItemToObject(root, "data", data_array);
    cJSON_AddNumberToObject(root, "type", type);
    cJSON_AddNumberToObject(root, "device", device);
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
int Read_Modbus_Rtu_Input_Registers(int address,int device,int num,int type){
    cJSON *root;
    char *data;
    char buffer[200];
    int res;
    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "command", "read_modbus_rtu_input_registers");
    cJSON_AddNumberToObject(root, "address", address);
    cJSON_AddNumberToObject(root, "device", device);
    cJSON_AddNumberToObject(root, "num", num);
    cJSON_AddNumberToObject(root, "type", type);
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
//查询机械臂当前状态
int Parser_Arm_Current_State(char *msg)
{
    cJSON *root = NULL, *arm_state, *joint, *pose, *arm_err, *err;
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
    if(arm_dof == 7)
    {
        Arm_Current_State.joint[6] = cJSON_GetArrayItem(joint, 6)->valueint;
    }

    err = cJSON_GetObjectItem(arm_state,"err");
 
    int Err_size = cJSON_GetArraySize(err);//可以获取长度
    Arm_Current_State.error.resize(Err_size);
    for(i = 0; i < Err_size; i++)
    {
        Arm_Current_State.error[i] = cJSON_GetArrayItem(err, i)->valueint;
        // ROS_INFO("Arm_Current_State.arm_err: %d", Arm_Current_State.error[i]);
    }

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
    int data[7];
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
    if (size == arm_dof)
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

int Parser_One_Force_Data(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);
    int data;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "Fz");
    if (result->type == cJSON_Number)
    {
        data = result->valueint;
        RM_Joint.six_force[1] = data;
        RM_Joint.six_force[1] = RM_Joint.six_force[1] / 1000;
        // ROS_INFO("RM_Joint.six_force[%d]:%f",i, RM_Joint.six_force[i]);
    }
    else
    {
        return 1;
    }
    result = cJSON_GetObjectItem(root, "zero_Fz");
    if (result->type == cJSON_Number)
    {
        data = result->valueint;
        RM_Joint.zero_force[1] = data;
        RM_Joint.zero_force[1] = RM_Joint.zero_force[1] / 1000;
        // ROS_INFO("RM_Joint.six_force[%d]:%f",i, RM_Joint.six_force[i]);
    }
    else
    {
        return 1;
    }
    result = cJSON_GetObjectItem(root, "work_zero_Fz");
    if (result->type == cJSON_Number)
    {
        data = result->valueint;
        RM_Joint.work_zero_force[1] = data;
        RM_Joint.work_zero_force[1] = RM_Joint.work_zero_force[1] / 1000;
        // ROS_INFO("RM_Joint.six_force[%d]:%f",i, RM_Joint.six_force[i]);
    }
    else
    {
        return 1;
    }
    result = cJSON_GetObjectItem(root, "tool_zero_Fz");
    if (result->type == cJSON_Number)
    {
        data = result->valueint;
        RM_Joint.tool_zero_force[1] = data;
        RM_Joint.tool_zero_force[1] = RM_Joint.tool_zero_force[1] / 1000;
        // ROS_INFO("RM_Joint.six_force[%d]:%f",i, RM_Joint.six_force[i]);
        cJSON_Delete(root);
        return 0;
    }
    else
    {
        return 1;
    }
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
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    result = cJSON_GetObjectItem(root, "zero_force_data");
    if (result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if (size == 6)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                data[i] = json_sub->valueint;
                RM_Joint.zero_force[i] = data[i];
                RM_Joint.zero_force[i] = RM_Joint.zero_force[i] / 1000;
                // ROS_INFO("RM_Joint.zero_force[%d]:%f",i, RM_Joint.zero_force[i]);
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
    result = cJSON_GetObjectItem(root, "work_zero_force_data");
    if (result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if (size == 6)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                data[i] = json_sub->valueint;
                RM_Joint.work_zero_force[i] = data[i];
                RM_Joint.work_zero_force[i] = RM_Joint.work_zero_force[i] / 1000;
                // ROS_INFO("RM_Joint.work_zero_force[%d]:%f",i, RM_Joint.work_zero_force[i]);
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
    result = cJSON_GetObjectItem(root, "tool_zero_force_data");
    if (result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if (size == 6)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                data[i] = json_sub->valueint;
                RM_Joint.tool_zero_force[i] = data[i];
                RM_Joint.tool_zero_force[i] = RM_Joint.tool_zero_force[i] / 1000;
                // ROS_INFO("RM_Joint.tool_zero_force[%d]:%f",i, RM_Joint.tool_zero_force[i]);
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
        if (size == arm_dof)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                RM_Joint.joint_current[i] = json_sub->valueint;
                RM_Joint.joint_current[i] = RM_Joint.joint_current[i] / 1000;
            }
            if(arm_dof == 7)
            {
                json_sub = cJSON_GetArrayItem(result, 6);
                RM_Joint.joint_current[6] = json_sub->valueint;
                RM_Joint.joint_current[6] = RM_Joint.joint_current[6] / 1000;
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

int Parser_Expand_State(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "pos");
    if (result != NULL && result->type == cJSON_Number)
    {
        Expand_Current_State.pos = result->valueint;
        result = cJSON_GetObjectItem(root, "err_flag");
        if (result != NULL && result->type == cJSON_Number)
        {
            Expand_Current_State.err_flag = result->valueint;
        }
        result = cJSON_GetObjectItem(root, "en_flag");
        if (result != NULL && result->type == cJSON_Number)
        {
            Expand_Current_State.en_flag = result->valueint;
        }
        result = cJSON_GetObjectItem(root, "current");
        if (result != NULL && result->type == cJSON_Number)
        {
            Expand_Current_State.current = result->valueint;
        }
        result = cJSON_GetObjectItem(root, "mode");
        if (result != NULL && result->type == cJSON_Number)
        {
            Expand_Current_State.mode = result->valueint;
        }
        result = cJSON_GetObjectItem(root, "joint_id");
        if (result != NULL && result->type == cJSON_Number)
        {
            Expand_Current_State.joint_id = result->valueint;
        }
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}
int Parser_Get_Realtime_Push_Data(char *msg)
{
    cJSON *root = NULL, *result, *custom;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    result = cJSON_GetObjectItem(root, "cycle");
    if (result != NULL && result->type == cJSON_Number)
    {
        Udp_Setting.udp_cycle = result->valueint;
        // result = cJSON_GetObjectItem(root, "enable");
        // if ((result->type == cJSON_True) || (result->type == cJSON_False))
        // {
        //     Udp_Setting.udp_enable = result->valueint;
        // }
        result = cJSON_GetObjectItem(root, "port");
        if (result != NULL && result->type == cJSON_Number)
        {
            Udp_Setting.udp_port = result->valueint;
        }
        result = cJSON_GetObjectItem(root, "force_coordinate");
        if (result != NULL && result->type == cJSON_Number)
        {
            Udp_Setting.udp_force_coordinate = result->valueint;
        }
        result = cJSON_GetObjectItem(root, "ip");
        if (result != NULL && result->type == cJSON_String)
        {
            Udp_Setting.udp_ip = result->valuestring;
        }
    }
    else
        return 3;
    custom = cJSON_GetObjectItem(root, "custom");
    if(custom != NULL && custom->type == cJSON_Object)    
    {
        result = cJSON_GetObjectItem(custom, "aloha_state");
        if(result != NULL)
        {
            if(result->type == cJSON_True)
                Udp_Setting.custom_set_data.aloha_state_ = true;
            else
                Udp_Setting.custom_set_data.aloha_state_ = false;
        }
        result = cJSON_GetObjectItem(custom, "expand_state");
        if(result != NULL)
        {
            if(result->type == cJSON_True)
                Udp_Setting.custom_set_data.expand_state_ = true;
            else
                Udp_Setting.custom_set_data.expand_state_ = false;
        }
        result = cJSON_GetObjectItem(custom, "joint_speed");
        if(result != NULL)
        {
            if(result->type == cJSON_True)
                Udp_Setting.custom_set_data.joint_speed_ = true;
            else
                Udp_Setting.custom_set_data.joint_speed_ = false;
        }
        result = cJSON_GetObjectItem(custom, "lift_state");
        if(result != NULL)
        {
            if(result->type == cJSON_True)
                Udp_Setting.custom_set_data.lift_state_ = true;
            else
                Udp_Setting.custom_set_data.lift_state_ = false;
        }
        result = cJSON_GetObjectItem(custom, "arm_current_status");
        if(result != NULL)
        {
            if(result->type == cJSON_True)
                Udp_Setting.custom_set_data.arm_current_status_ = true;
            else
                Udp_Setting.custom_set_data.arm_current_status_ = false;
        }
        result = cJSON_GetObjectItem(custom, "joint_acc");
        if(result != NULL)
        {
            if(result->type == cJSON_True)
                Udp_Setting.custom_set_data.joint_acc_ = true;
            else
                Udp_Setting.custom_set_data.joint_acc_ = false;
        }
        result = cJSON_GetObjectItem(custom, "tail_end");
        if(result != NULL)
        {
            if(result->type == cJSON_True)
                Udp_Setting.custom_set_data.tail_end_ = true;
            else
                Udp_Setting.custom_set_data.tail_end_ = false;
        }
        result = cJSON_GetObjectItem(custom, "rm_plus_base");
        if(result != NULL)
        {
            if(result->type == cJSON_True)
                Udp_Setting.custom_set_data.rm_plus_base_ = true;
            else
                Udp_Setting.custom_set_data.rm_plus_base_ = false;
        }
        result = cJSON_GetObjectItem(custom, "rm_plus_state");
        if(result != NULL)
        {
            if(result->type == cJSON_True)
                Udp_Setting.custom_set_data.rm_plus_state_ = true;
            else
                Udp_Setting.custom_set_data.rm_plus_state_ = false;
        }
    }
    cJSON_Delete(root);
    return 0;
}

// Udp Parser Realtime_Arm_Joint_State
int Parser_Realtime_Arm_Joint_State(char *msg)
{
    cJSON *root = NULL, *result, *json_member, *json_sub;
    root = cJSON_Parse(msg);
    int i = 0;
    int data[7];
    int size = 0;
    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
/***********************************arm_current_status状态**************************************************/
    result = cJSON_GetObjectItem(root, "arm_current_status");
    if(result != NULL)
    {
        Udp_RM_Joint.arm_current_status = result->valuestring;
        udp_arm_current_status.arm_current_status = Udp_RM_Joint.arm_current_status; 
        pub_ArmCurrentStatus.publish(udp_arm_current_status);
    }
/*************************************joint_status相关数据**************************************************/
    result = cJSON_GetObjectItem(root, "joint_status");
    /**********************获取电流数据******************************/
    json_member = cJSON_GetObjectItem(result, "joint_current");
    if (json_member != NULL && json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == arm_dof)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(json_member, i);
                Udp_RM_Joint.joint_current[i] = json_sub->valueint;
                Udp_RM_Joint.joint_current[i] = Udp_RM_Joint.joint_current[i] / 1000;
                //ROS_INFO("Udp_RM_Joint.joint_current %d is %f ",i,Udp_RM_Joint.joint_current[i]);
                udp_joint_current.joint_current[i]=Udp_RM_Joint.joint_current[i];
            }
            pub_JointCurrent.publish(udp_joint_current);
            // cJSON_Delete(root);
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    /***************************************************************/

    /**********************获取机械臂关节使能状态************************/
    json_member = cJSON_GetObjectItem(result, "joint_en_flag");
    if (json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == arm_dof)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(json_member, i);
                data[i] = json_sub->valueint;
                Udp_RM_Joint.en_flag[i] = (uint16_t)(data[i]);
                //ROS_INFO("Udp_RM_Joint.en_flag %d is %d ",i,Udp_RM_Joint.en_flag[i]);
                udp_joint_en_flag.joint_en_flag[i]=Udp_RM_Joint.en_flag[i];
            } 
            pub_JointEnFlag.publish(udp_joint_en_flag);
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    /****************************************************************/

    /**********************获取机械臂关节错误码*************************/
    json_member = cJSON_GetObjectItem(result, "joint_err_code");
    if (json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == arm_dof)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(json_member, i);
                data[i] = json_sub->valueint;
                Udp_RM_Joint.err_flag[i] = (uint16_t)(data[i]);
                udp_joint_error_code.joint[i] = Udp_RM_Joint.err_flag[i];
                // ROS_INFO("Udp_RM_Joint.err_flag %d is %d ",i,Udp_RM_Joint.err_flag[i]);
            }
            pub_JointErrorCode.publish(udp_joint_error_code);
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }

    /***********************获取机械臂关节角度*************************/
    json_member = cJSON_GetObjectItem(result, "joint_position");
    if (json_member != NULL && json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == arm_dof)
        {
            for (i = 0; i < size; i++)
            {   
                data[i] = Udp_RM_Joint.joint[i] = udp_real_joint.position[i] = 0;
                json_sub = cJSON_GetArrayItem(json_member, i);
                data[i] = json_sub->valueint;
                Udp_RM_Joint.joint[i] = data[i];
                Udp_RM_Joint.joint[i] = Udp_RM_Joint.joint[i] / 1000;
            }
            udp_real_joint.header.stamp = ros::Time::now();
            for (i = 0; i < size; i++)
            {
                udp_real_joint.position[i] = Udp_RM_Joint.joint[i] * DEGREE_RAD;
                // ROS_INFO("Udp_RM_Joint.joint %d is %f ",i,udp_real_joint.position[i]);
            }
            Joint_State.publish(udp_real_joint);
            Info_Arm_Err();
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    /******************************************************************/

    /***********************获取机械臂关节速度****************************/
    json_member = cJSON_GetObjectItem(result, "joint_speed");
    if (json_member != NULL && json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == arm_dof)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(json_member, i);
                data[i] = json_sub->valueint;
                Udp_RM_Joint.joint_speed[i] = data[i];
                Udp_RM_Joint.joint_speed[i] = Udp_RM_Joint.joint_speed[i] / 50;
                udp_joint_speed.joint_speed[i]= Udp_RM_Joint.joint_speed[i];
                // ROS_INFO("Udp_RM_Joint.joint_temperature %d is %f ",i,Udp_RM_Joint.joint_temperature[i]);
            }
            pub_JointSpeed.publish(udp_joint_speed);
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    // else
    // {
    //     cJSON_Delete(root);
    //     return 1;
    // }
    /***********************************************************************/

    /***********************获取机械臂关节温度****************************/
    json_member = cJSON_GetObjectItem(result, "joint_temperature");
    if (json_member != NULL && json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == arm_dof)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(json_member, i);
                data[i] = json_sub->valueint;
                Udp_RM_Joint.joint_temperature[i] = data[i];
                Udp_RM_Joint.joint_temperature[i] = Udp_RM_Joint.joint_temperature[i] / 1000;
                // ROS_INFO("Udp_RM_Joint.joint_temperature %d is %f ",i,Udp_RM_Joint.joint_temperature[i]);
                udp_joint_temperature.joint_temperature[i]=Udp_RM_Joint.joint_temperature[i]; 
            }
            pub_JointTemperature.publish(udp_joint_temperature);
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    /***********************************************************************/

    /***********************获取机械臂关节电压*************************/
    json_member = cJSON_GetObjectItem(result, "joint_voltage");
    if (json_member != NULL && json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == arm_dof)
        {
            for (i = 0; i < size; i++)
            {
                json_sub = cJSON_GetArrayItem(json_member, i);
                data[i] = json_sub->valueint;
                Udp_RM_Joint.joint_voltage[i] = data[i];
                Udp_RM_Joint.joint_voltage[i] = Udp_RM_Joint.joint_voltage[i] / 1000;
                // ROS_INFO("Udp_RM_Joint.joint_voltage %d is %f ",i,Udp_RM_Joint.joint_voltage[i]);
                udp_joint_voltage.joint_voltage[i]=Udp_RM_Joint.joint_voltage[i]; 
            }
            pub_JointVoltage.publish(udp_joint_voltage);
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    /***********************************************************************/
/**********************************************************************************************************/

/**********************************************灵巧手state相关数据******************************************************/
    result = cJSON_GetObjectItem(root, "hand");
    if(result != NULL)
    {
        /*****************************获取err信息***************************/
        json_member = cJSON_GetObjectItem(result, "hand_err");
        if (json_member != NULL )
        {
            Udp_Hand_Data.hand_err = json_member->valueint;
            udp_hand_status.hand_err = Udp_Hand_Data.hand_err;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***********************************************************************/

        /***************************获取hand_pos信息********************************/
        json_member = cJSON_GetObjectItem(result, "hand_pos");
        
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            if (size == 6)
            {
                for (i = 0; i < 6; i++)
                {
                    json_sub = cJSON_GetArrayItem(json_member, i);
                    data[i] = json_sub->valueint;
                    Udp_Hand_Data.hand_pos[i] = data[i];
                    udp_hand_status.hand_pos[i] = Udp_Hand_Data.hand_pos[i];
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
            cJSON_Delete(root);
            return 1;
        }

        /***************************获取hand_angle信息********************************/
        json_member = cJSON_GetObjectItem(result, "hand_angle");

        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            if (size == 6)
            {
                for (i = 0; i < 6; i++)
                {
                    json_sub = cJSON_GetArrayItem(json_member, i);
                    data[i] = json_sub->valueint;
                    Udp_Hand_Data.hand_angle[i] = data[i];
                    udp_hand_status.hand_angle[i] = Udp_Hand_Data.hand_angle[i];
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
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取hand_force信息********************************/
        json_member = cJSON_GetObjectItem(result, "hand_force");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            if (size == 6)
            {
                for (i = 0; i < 6; i++)
                {
                    json_sub = cJSON_GetArrayItem(json_member, i);
                    data[i] = json_sub->valueint;
                    Udp_Hand_Data.hand_force[i] = data[i];
                    udp_hand_status.hand_force[i] = Udp_Hand_Data.hand_force[i];
                    
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
            cJSON_Delete(root);
            return 1;
        }

        /***************************获取hand_state信息********************************/
        json_member = cJSON_GetObjectItem(result, "hand_state");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            if (size == 6)
            {
                for (i = 0; i < 6; i++)
                {
                    json_sub = cJSON_GetArrayItem(json_member, i);
                    data[i] = json_sub->valueint;
                    Udp_Hand_Data.hand_state[i] = data[i];
                    udp_hand_status.hand_state[i] = Udp_Hand_Data.hand_state[i];
                }
                pub_HandStatus.publish(udp_hand_status);
            }

            else
            {
                cJSON_Delete(root);
                return 3;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
    }
/********************************************************************************************** */
    result = cJSON_GetObjectItem(root, "rm_plus_state");
    if((result != NULL)&&(strcmp("disable", result->valuestring)))
    {
        /*****************************获取sys_state信息***************************/
        json_member = cJSON_GetObjectItem(result, "sys_state");
        if (json_member != NULL )
        {
            udp_plus_state.sys_state = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取sys_err信息***************************/
        json_member = cJSON_GetObjectItem(result, "sys_err");
        if (json_member != NULL )
        {
            udp_plus_state.sys_err = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取angle信息***************************/
        json_member = cJSON_GetObjectItem(result, "angle");
        if (json_member != NULL && json_member->type == cJSON_Array )
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.angle.resize(size);
            for(int i = 0;i<size;i++)
            {
                udp_plus_state.angle[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
            // Udp_Hand_Data.hand_err = json_member->valueint;
            // udp_plus_state.hand_err = Udp_Hand_Data.hand_err;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***********************************************************************/

        /***************************获取hand_pos信息********************************/
        json_member = cJSON_GetObjectItem(result, "current");
        
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.current.resize(size);
            for (i = 0; i < size; i++)
            {
                udp_plus_state.current[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }

        /***************************获取hand_angle信息********************************/
        json_member = cJSON_GetObjectItem(result, "dof_err");

        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.dof_err.resize(size);
            for (i = 0; i < size; i++)
            {
                udp_plus_state.dof_err[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取dof_state信息********************************/
        json_member = cJSON_GetObjectItem(result, "dof_state");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.dof_state.resize(size);
            for (i = 0; i < size; i++)
            {
                udp_plus_state.dof_state[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }

        /***************************获取normal_force信息********************************/
        json_member = cJSON_GetObjectItem(result, "normal_force");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.normal_force.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_state.normal_force[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取pos信息********************************/
        json_member = cJSON_GetObjectItem(result, "pos");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.pos.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_state.pos[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取speed信息********************************/
        json_member = cJSON_GetObjectItem(result, "speed");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.speed.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_state.speed[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取tangential_force信息********************************/
        json_member = cJSON_GetObjectItem(result, "tangential_force");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.tangential_force.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_state.tangential_force[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取tangential_force_dir信息********************************/
        json_member = cJSON_GetObjectItem(result, "tangential_force_dir");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.tangential_force_dir.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_state.tangential_force_dir[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取tma信息********************************/
        json_member = cJSON_GetObjectItem(result, "tma");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.tma.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_state.tma[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取tsa信息********************************/
        json_member = cJSON_GetObjectItem(result, "tsa");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_state.tsa.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_state.tsa[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
            pub_RmPlusState.publish(udp_plus_state);
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
    }
/********************************************rm_plus_base******************************************/
    result = cJSON_GetObjectItem(root, "rm_plus_base");
    if((result != NULL)&&(strcmp("disable", result->valuestring)))
    {
        /*****************************获取bee信息***************************/
        json_member = cJSON_GetObjectItem(result, "bee");
        if (json_member != NULL )
        {
            udp_plus_base.bee = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取bv信息***************************/
        json_member = cJSON_GetObjectItem(result, "bv");
        if (json_member != NULL )
        {
            udp_plus_base.bv = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取check信息***************************/
        json_member = cJSON_GetObjectItem(result, "check");
        if (json_member != NULL )
        {
            udp_plus_base.check = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取dof信息***************************/
        json_member = cJSON_GetObjectItem(result, "dof");
        if (json_member != NULL )
        {
            udp_plus_base.dof = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取force信息***************************/
        json_member = cJSON_GetObjectItem(result, "force");
        if ( json_member != NULL )
        {
            if(json_member->type == cJSON_True)
            udp_plus_base.force = true;
            else if(json_member->type == cJSON_False)
            udp_plus_base.force = false;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取hand信息***************************/
        json_member = cJSON_GetObjectItem(result, "hand");
        if (json_member != NULL )
        {
            udp_plus_base.hand = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取hv信息***************************/
        json_member = cJSON_GetObjectItem(result, "hv");
        if (json_member != NULL )
        {
            udp_plus_base.hv = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取id信息***************************/
        json_member = cJSON_GetObjectItem(result, "id");
        if (json_member != NULL )
        {
            udp_plus_base.id = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取manu信息***************************/
        json_member = cJSON_GetObjectItem(result, "manu");
        if (json_member != NULL )
        {
            udp_plus_base.manu = json_member->valuestring;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取sv信息***************************/
        json_member = cJSON_GetObjectItem(result, "sv");
        if (json_member != NULL )
        {
            udp_plus_base.sv = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取force信息***************************/
        json_member = cJSON_GetObjectItem(result, "touch");
        if ( json_member != NULL )
        {
            if(json_member->type == cJSON_True)
            udp_plus_base.touch = true;
            else if(json_member->type == cJSON_False)
            udp_plus_base.touch = false;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取touch_num信息***************************/
        json_member = cJSON_GetObjectItem(result, "touch_num");
        if (json_member != NULL )
        {
            udp_plus_base.touch_num = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取touch_sw信息***************************/
        json_member = cJSON_GetObjectItem(result, "touch_sw");
        if (json_member != NULL )
        {
            udp_plus_base.touch_sw = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取type信息***************************/
        json_member = cJSON_GetObjectItem(result, "type");
        if (json_member != NULL )
        {
            udp_plus_base.type = json_member->valueint;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /*****************************获取angle信息***************************/
        json_member = cJSON_GetObjectItem(result, "angle_low");
        if (json_member != NULL && json_member->type == cJSON_Array )
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_base.angle_low.resize(size);
            for(int i = 0;i<size;i++)
            {
                udp_plus_base.angle_low[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
            // Udp_Hand_Data.hand_err = json_member->valueint;
            // udp_hand_status.hand_err = Udp_Hand_Data.hand_err;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***********************************************************************/

        /*****************************获取angle信息***************************/
        json_member = cJSON_GetObjectItem(result, "angle_up");
        if (json_member != NULL && json_member->type == cJSON_Array )
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_base.angle_up.resize(size);
            for(int i = 0;i<size;i++)
            {
                udp_plus_base.angle_up[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
            // Udp_Hand_Data.hand_err = json_member->valueint;
            // udp_hand_status.hand_err = Udp_Hand_Data.hand_err;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***********************************************************************/
        /*****************************获取force_low信息***************************/
        json_member = cJSON_GetObjectItem(result, "force_low");
        if (json_member != NULL && json_member->type == cJSON_Array )
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_base.force_low.resize(size);
            for(int i = 0;i<size;i++)
            {
                udp_plus_base.force_low[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
            // Udp_Hand_Data.hand_err = json_member->valueint;
            // udp_hand_status.hand_err = Udp_Hand_Data.hand_err;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***********************************************************************/
        /*****************************获取force信息***************************/
        json_member = cJSON_GetObjectItem(result, "force_up");
        if (json_member != NULL && json_member->type == cJSON_Array )
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_base.force_up.resize(size);
            for(int i = 0;i<size;i++)
            {
                udp_plus_base.force_up[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
            // Udp_Hand_Data.hand_err = json_member->valueint;
            // udp_hand_status.hand_err = Udp_Hand_Data.hand_err;
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***********************************************************************/
        /***************************获取pos信息********************************/
        json_member = cJSON_GetObjectItem(result, "pos_low");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_base.pos_low.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_base.pos_low[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取pos信息********************************/
        json_member = cJSON_GetObjectItem(result, "pos_up");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_base.pos_up.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_base.pos_up[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取speed_low信息********************************/
        json_member = cJSON_GetObjectItem(result, "speed_low");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_base.speed_low.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_base.speed_low[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
        /***************************获取speed_up信息********************************/
        json_member = cJSON_GetObjectItem(result, "speed_up");
        if (json_member != NULL && json_member->type == cJSON_Array)
        {
            size = cJSON_GetArraySize(json_member);
            udp_plus_base.speed_up.resize(size);

            for (i = 0; i < size; i++)
            {
                udp_plus_base.speed_up[i] = cJSON_GetArrayItem(json_member, i)->valueint;
            }
            pub_RmPlusBase.publish(udp_plus_base);
        }
        else
        {
            cJSON_Delete(root);
            return 1;
        }
    }


/**********************************************state相关数据******************************************************/
    result = cJSON_GetObjectItem(root, "waypoint");
    /*****************************获取position信息***************************/
    json_member = cJSON_GetObjectItem(result, "position");
    if (json_member != NULL && json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == 3)
        {
            for (i = 0; i < 3; i++)
            {
                json_sub = cJSON_GetArrayItem(json_member, i);
                data[i] = json_sub->valueint;
                Udp_RM_Joint.joint_position[i] = data[i];
                Udp_RM_Joint.joint_position[i] = Udp_RM_Joint.joint_position[i] / 1000000;
                udp_joint_poseEuler.position[i]=Udp_RM_Joint.joint_position[i];
                // ROS_INFO("Arm_Current_position.joint_position[%d] : %f",i,Udp_RM_Joint.joint_position[i]);
            }
            udp_arm_pose.position.x = Udp_RM_Joint.joint_position[0];
            udp_arm_pose.position.y = Udp_RM_Joint.joint_position[1];
            udp_arm_pose.position.z = Udp_RM_Joint.joint_position[2];
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    /***********************************************************************/

    /***************************获取欧拉角信息********************************/
    json_member = cJSON_GetObjectItem(result, "euler");
    if (json_member != NULL && json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == 3)
        {
            for (i = 0; i < 3; i++)
            {
                json_sub = cJSON_GetArrayItem(json_member, i);
                data[i] = json_sub->valueint;
                Udp_RM_Joint.joint_euler[i] = data[i];
                Udp_RM_Joint.joint_euler[i] = Udp_RM_Joint.joint_euler[i] / 1000;
                udp_joint_poseEuler.euler[i] = Udp_RM_Joint.joint_euler[i];
                // ROS_INFO("Arm_Current_position.joint_euler[%d] : %f",i,Udp_RM_Joint.joint_euler[i]);
            }
            pub_PoseEuler.publish(udp_joint_poseEuler);
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    /*************************************************************************/

    /***************************获取四元数信息********************************/
    json_member = cJSON_GetObjectItem(result, "quat");
    if (json_member != NULL && json_member->type == cJSON_Array)
    {
        size = cJSON_GetArraySize(json_member);
        if (size == 4)
        {
            for (i = 0; i < 4; i++)
            {
                json_sub = cJSON_GetArrayItem(json_member, i);
                data[i] = json_sub->valueint;
                Udp_RM_Joint.joint_quat[i] = data[i];
                Udp_RM_Joint.joint_quat[i] = Udp_RM_Joint.joint_quat[i] / 1000000;
                // ROS_INFO("Arm_Current_position.joint_quat[%d] : %f",i,Udp_RM_Joint.joint_quat[i]);
            }
            udp_arm_pose.orientation.x = Udp_RM_Joint.joint_quat[1];
            udp_arm_pose.orientation.y = Udp_RM_Joint.joint_quat[2];
            udp_arm_pose.orientation.z = Udp_RM_Joint.joint_quat[3];
            udp_arm_pose.orientation.w = Udp_RM_Joint.joint_quat[0];
            pub_PoseState.publish(udp_arm_pose);
        }
        else
        {
            cJSON_Delete(root);
            return 3;
        }

    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    /*************************************************************************/
/*************************************************************************************************************************/


/**********************************************six_force_sensor六维力相关数据************************************************/
    if((force_sensor == 1))
    {
        result = cJSON_GetObjectItem(root, "six_force_sensor");
        if(result!= NULL)
        {
            /**************获取force信息*当前力传感器原始数据0.001N或0.001Nm**********/
            json_member = cJSON_GetObjectItem(result, "force");
            if (json_member != NULL && json_member->type == cJSON_Array)
            {
                size = cJSON_GetArraySize(json_member);
                if (size == 6)
                {
                    for (i = 0; i < 6; i++)
                    {
                        json_sub = cJSON_GetArrayItem(json_member, i);
                        data[i] = json_sub->valueint;
                        Udp_RM_Joint.six_force[i] = data[i];
                        Udp_RM_Joint.six_force[i] = Udp_RM_Joint.six_force[i] / 1000;
                        // ROS_INFO("Arm_Current_position.six_force[%d] : %f",i,Udp_RM_Joint.six_force[i]);
                    }
                    Udp_Six_Force.force_Fx = Udp_RM_Joint.six_force[0];
                    Udp_Six_Force.force_Fy = Udp_RM_Joint.six_force[1];
                    Udp_Six_Force.force_Fz = Udp_RM_Joint.six_force[2];
                    Udp_Six_Force.force_Mx = Udp_RM_Joint.six_force[3];
                    Udp_Six_Force.force_My = Udp_RM_Joint.six_force[4];
                    Udp_Six_Force.force_Mz = Udp_RM_Joint.six_force[5];
                    pub_UdpSixForce.publish(Udp_Six_Force);
                }
                else
                {
                    cJSON_Delete(root);
                    return 3;
                }
            }
            else
            {
                cJSON_Delete(root);
                return 1;
            }
            /***********************************************************************/

            /***********zero_force*当前力传感器系统外受力数据0.001N或0.001Nm************/
            json_member = cJSON_GetObjectItem(result, "zero_force");
            if (json_member != NULL && json_member->type == cJSON_Array)
            {
                size = cJSON_GetArraySize(json_member);
                if (size == 6)
                {
                    for (i = 0; i < 6; i++)
                    {
                        json_sub = cJSON_GetArrayItem(json_member, i);
                        data[i] = json_sub->valueint;
                        Udp_RM_Joint.zero_force[i] = data[i];
                        Udp_RM_Joint.zero_force[i] = Udp_RM_Joint.zero_force[i] / 1000;
                        // ROS_INFO("Arm_Current_position.zero_force[%d] : %f",i,Udp_RM_Joint.zero_force[i]);
                    }
                    Udp_Six_Zero_Force.force_Fx = Udp_RM_Joint.zero_force[0];
                    Udp_Six_Zero_Force.force_Fy = Udp_RM_Joint.zero_force[1];
                    Udp_Six_Zero_Force.force_Fz = Udp_RM_Joint.zero_force[2];
                    Udp_Six_Zero_Force.force_Mx = Udp_RM_Joint.zero_force[3];
                    Udp_Six_Zero_Force.force_My = Udp_RM_Joint.zero_force[4];
                    Udp_Six_Zero_Force.force_Mz = Udp_RM_Joint.zero_force[5];
                    pub_UdpSixZeroForce.publish(Udp_Six_Zero_Force);
                }
                else
                {
                    cJSON_Delete(root);
                    return 3;
                }
            }
            else
            {
                cJSON_Delete(root);
                return 1;
            }
            /***********************************************************************/

            /*************************得到当前的相对坐标信息**************************/
            json_member = cJSON_GetObjectItem(result, "coordinate");
            if (json_member != NULL && json_member->type == cJSON_Number)
            {
                udp_coordinate.data = json_member->valueint;
                pub_Udp_Coordinate.publish(udp_coordinate);
                // ROS_INFO("arm_err = %d",arm_err);
            }
            else
            {
                return 1;
            }
            /***********************************************************************/
        }

        else
        {
            if(force_sensor == 0)
            {;}
            else
            {ROS_ERROR("Six_Force_Sensor Error");}
        }
    }
/*************************************************************************************************************************/


/**********************************************one_force_sensor一维力相关数据************************************************/
    if((force_sensor == 2))
    {
        result = cJSON_GetObjectItem(root, "one_force_sensor");
        if(result!= NULL)
        {
            /**************获取force信息*当前力传感器原始数据0.001N或0.001Nm**********/
            json_member = cJSON_GetObjectItem(result, "force");
            if (json_member != NULL && json_member->type == cJSON_Array)
            {
                json_sub = cJSON_GetArrayItem(json_member, 0);
                Udp_RM_Joint.force = json_sub->valueint;
                Udp_RM_Joint.force = Udp_RM_Joint.force / 1000.0;
                // ROS_INFO("Udp_RM_Joint.force = %d",Udp_RM_Joint.force);
                Udp_Six_Force.force_Fz = Udp_RM_Joint.force;
                pub_UdpSixForce.publish(Udp_Six_Force);
            }
            
            else
            {
                cJSON_Delete(root);
                return 1;
            }
        
            /***********************************************************************/

            /***********zero_force*当前力传感器系统外受力数据0.001N或0.001Nm************/
            json_member = cJSON_GetObjectItem(result, "zero_force");
            if (json_member != NULL && json_member->type == cJSON_Array)
            {
                json_sub = cJSON_GetArrayItem(json_member, 0);
                Udp_RM_Joint.joint_zero_force = json_sub->valueint;
                Udp_RM_Joint.joint_zero_force = Udp_RM_Joint.joint_zero_force/1000.0;
                // ROS_INFO("Udp_RM_Joint.joint_zero_force = %d",Udp_RM_Joint.joint_zero_force);
                Udp_Six_Zero_Force.force_Fz = Udp_RM_Joint.joint_zero_force;
                pub_UdpSixZeroForce.publish(Udp_Six_Zero_Force);
            }
            else
            {
                cJSON_Delete(root);
                return 1;
            }
            /***********************************************************************/

            /*************************得到当前的相对坐标信息**************************/
            json_member = cJSON_GetObjectItem(result, "coordinate");
            if (json_member != NULL && json_member->type == cJSON_Number)
            {
                udp_coordinate.data = json_member->valueint;
                pub_Udp_Coordinate.publish(udp_coordinate);
                // ROS_INFO("arm_err = %d",arm_err);
            }
            else
            {
                cJSON_Delete(root);
                return 1;
            }
            /***********************************************************************/
        }
        else
        {
            if(force_sensor == 0)
            {force_sensor = 12;}
            else
            {ROS_ERROR("One_Force_Sensor Error");}
        }
    }
    udp_failed_time = 0;
    cJSON_Delete(root);
    return 0;

}

int Parser_Udp_Msg(char *msg)
{
    cJSON *root = NULL, *json_state;
    root = cJSON_Parse(msg);

    int res = 0;

    if (root == NULL)
    {
        cJSON_Delete(root);
        return -1;
    }
   
    /**********************************处理arm_err*************************/
    json_state = cJSON_GetObjectItem(root,"err");
    if (json_state != NULL && json_state->type == cJSON_Array)
    {
        int Err_size = cJSON_GetArraySize(json_state);//可以获取长度
        udp_error.err.resize(Err_size);
        for(int i = 0; i < Err_size; i++)
        {
            udp_error.err[i] = cJSON_GetArrayItem(json_state, i)->valueint;
            // ROS_INFO("Arm_Current_State.arm_err: %d", Arm_Current_State.error[i]);
        }
        pub_UdpError.publish(udp_error);
    }
    else
    {
        cJSON_Delete(root);
        return 1;
    }
    // json_state = cJSON_GetObjectItem(root, "arm_err");
    // if (json_state != NULL && json_state->type == cJSON_Number)
    // {
    //     arm_err = json_state->valueint;
    //     udp_arm_error.data = arm_err;
    //     pub_ArmError.publish(udp_arm_error);
    //     // ROS_INFO("arm_err = %d",arm_err);
    // }
    
    
    /**********************************处理sys_err*************************/
    // json_state = cJSON_GetObjectItem(root, "sys_err");
    // if (json_state != NULL && json_state->type == cJSON_Number)
    // {
    //     sys_err = json_state->valueint;
    //     udp_sys_error.data = sys_err;
    //     pub_SysError.publish(udp_sys_error);
    //     // ROS_INFO("sys_err = %d",sys_err);
    // }
    // else
    // {
    //     cJSON_Delete(root);
    //     return 1;
    // }
    
    /************************************处理UDP其他数据*****************************/
    json_state = cJSON_GetObjectItem(root, "joint_status");
    if (json_state != NULL)
    {
        res = Parser_Realtime_Arm_Joint_State(msg);
        if (res == 0)
        {
            cJSON_Delete(root);
            return 0;
        }
        else
        {
            cJSON_Delete(root);
            return -3;
        }
    }

    return 5;    
}

int Parser_Trajectory_List(char *msg)
{
    cJSON *root = NULL, *result, *list;
    root = cJSON_Parse(msg);

    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    trajectory_list = rm_msgs::Trajectorylist();
    result = cJSON_GetObjectItem(root, "page_num");
    if (result != NULL && result->type == cJSON_Number)
    {
        trajectory_list.page_num = result->valueint;

        result = cJSON_GetObjectItem(root, "page_size");
        if (result != NULL && result->type == cJSON_Number)
        {
            trajectory_list.page_size = result->valueint;
        }

        result = cJSON_GetObjectItem(root, "total_size");
        if (result != NULL && result->type == cJSON_Number)
        {
            trajectory_list.total_size = result->valueint;
        }

        result = cJSON_GetObjectItem(root, "vague_search");
        if (result != NULL && result->type == cJSON_String)
        {
            trajectory_list.vague_search = result->valuestring;
        }
        
        list = cJSON_GetObjectItem(root, "list");
        if (list != NULL) {
            int array_size = cJSON_GetArraySize(list);
            for (int i = 0; i < array_size; i++) {
                cJSON* item = cJSON_GetArrayItem(list, i);
                cJSON* time = cJSON_GetObjectItem(item, "create_time");
                cJSON* name = cJSON_GetObjectItem(item, "name");
                cJSON* points = cJSON_GetObjectItem(item, "point_num");
                
                if (time && name && points) {
                    rm_msgs::Trajectoryinfo trajectory_info;
                    trajectory_info.create_time = time->valuestring;
                    trajectory_info.name = name->valuestring;
                    trajectory_info.point_num = points->valueint;
                    trajectory_list.tra_list.push_back(trajectory_info);
                }
            }
        }
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Flowchart_Runstate(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);
    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    result = cJSON_GetObjectItem(root, "run_state");
    if (result != NULL && result->type == cJSON_Number)
    {
        flowchart_runstate.run_state = result->valueint;
        result = cJSON_GetObjectItem(root, "id");
        if (result != NULL && result->type == cJSON_Number)
        {
            flowchart_runstate.id = result->valueint;
        }
        result = cJSON_GetObjectItem(root, "name");
        if (result != NULL && result->type == cJSON_String)
        {
            flowchart_runstate.name= result->valuestring;
            ROS_INFO("当前使能的流程图编程文件名称: %s", flowchart_runstate.name.data());
        }
        result = cJSON_GetObjectItem(root, "plan_speed");
        if (result != NULL && result->type == cJSON_Number)
        {
            flowchart_runstate.plan_speed = result->valueint;
        }
        result = cJSON_GetObjectItem(root, "step_mode");
        if (result != NULL && result->type == cJSON_Number)
        {
            flowchart_runstate.step_mode = result->valueint;
        }
        result = cJSON_GetObjectItem(root, "modal_id");
        if (result != NULL && result->type == cJSON_String)
        {
            flowchart_runstate.modal_id = result->valuestring;
        }
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Arm_Software_Info(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);
    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    // std::cout << "Parser_Arm_Software_Info!" << std::endl;
    result = cJSON_GetObjectItem(root, "robot_controller_version");
    if (result != NULL && result->type == cJSON_String)
    {
        if(strcmp(result->valuestring, "4.0") == 0){
            arm_version.controller_version = result->valuestring;
            result = cJSON_GetObjectItem(root, "Product_version");
            if (result != NULL && result->type == cJSON_String)
            {
                arm_version.product_version = result->valuestring;
            }
            result = cJSON_GetObjectItem(root, "algorithm_info");
            if (result != NULL && result->type == cJSON_Object)
            {
                cJSON *version = cJSON_GetObjectItem(result, "version");
                if (version != NULL && version->type == cJSON_String)
                    arm_version.algorithm_info= version->valuestring;
            }
            result = cJSON_GetObjectItem(root, "communication_info");
            if (result != NULL && result->type == cJSON_Object)
            {
                cJSON *buildinfo = cJSON_GetObjectItem(result, "build_time");
                if (buildinfo != NULL && buildinfo->type == cJSON_String)
                    arm_version.com_info.build_time = buildinfo->valuestring;
                buildinfo = cJSON_GetObjectItem(result, "version");
                if (buildinfo != NULL && buildinfo->type == cJSON_String)
                    arm_version.com_info.version = buildinfo->valuestring;
            }
            result = cJSON_GetObjectItem(root, "ctrl_info");
            if (result != NULL && result->type == cJSON_Object)
            {
                cJSON *buildinfo = cJSON_GetObjectItem(result, "build_time");
                if (buildinfo != NULL && buildinfo->type == cJSON_String)
                    arm_version.ctrl_info.build_time = buildinfo->valuestring;
                buildinfo = cJSON_GetObjectItem(result, "version");
                if (buildinfo != NULL && buildinfo->type == cJSON_String)
                    arm_version.ctrl_info.version = buildinfo->valuestring;
            }
            result = cJSON_GetObjectItem(root, "program_info");
            if (result != NULL && result->type == cJSON_Object)
            {
                cJSON *buildinfo = cJSON_GetObjectItem(result, "build_time");
                if (buildinfo != NULL && buildinfo->type == cJSON_String)
                    arm_version.program_info.build_time = buildinfo->valuestring;
                buildinfo = cJSON_GetObjectItem(result, "version");
                if (buildinfo != NULL && buildinfo->type == cJSON_String)
                    arm_version.program_info.version = buildinfo->valuestring;
            }
        }
        cJSON_Delete(root);
        return 0;
    }
    result = cJSON_GetObjectItem(root, "plan_info");
    if (result != NULL && result->type == cJSON_Object)
    {
        cJSON *buildinfo = cJSON_GetObjectItem(result, "build_time");
        if (buildinfo != NULL && buildinfo->type == cJSON_String)
            arm_version.plan_info.build_time = buildinfo->valuestring;
        buildinfo = cJSON_GetObjectItem(result, "version");
        if (buildinfo != NULL && buildinfo->type == cJSON_String)
            arm_version.plan_info.version = buildinfo->valuestring;
        result = cJSON_GetObjectItem(root, "ctrl_info");
        if (result != NULL && result->type == cJSON_Object)
        {
            cJSON *buildinfo = cJSON_GetObjectItem(result, "build_time");
            if (buildinfo != NULL && buildinfo->type == cJSON_String)
                arm_version.ctrl_info.build_time = buildinfo->valuestring;
            buildinfo = cJSON_GetObjectItem(result, "version");
            if (buildinfo != NULL && buildinfo->type == cJSON_String)
                arm_version.ctrl_info.version = buildinfo->valuestring;
        }
        result = cJSON_GetObjectItem(root, "Product_version");
        if (result != NULL && result->type == cJSON_String)
        {
            arm_version.product_version = result->valuestring;
        }
        result = cJSON_GetObjectItem(root, "algorithm_info");
        if (result != NULL && result->type == cJSON_Object)
        {
            cJSON *version = cJSON_GetObjectItem(result, "version");
            if (version != NULL && version->type == cJSON_String)
            arm_version.algorithm_info= version->valuestring;
        }
        result = cJSON_GetObjectItem(root, "dynamic_info");
        if (result != NULL && result->type == cJSON_Object)
        {
            cJSON *version = cJSON_GetObjectItem(result, "model_version");
            if (version != NULL && version->type == cJSON_String)
            arm_version.algorithm_info= version->valuestring;
        }
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Get_Modbus_Tcp_Master(char *msg)
{
    cJSON *root = NULL, *result;
    root = cJSON_Parse(msg);
    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    result = cJSON_GetObjectItem(root, "master_name");
    if (result != NULL && result->type == cJSON_String)
    {
        modbustcp_master_info.master_name = result->valuestring;
        result = cJSON_GetObjectItem(root, "ip");
        if (result != NULL && result->type == cJSON_String)
        {
            modbustcp_master_info.ip= result->valuestring;
        }
        result = cJSON_GetObjectItem(root, "port");
        if (result != NULL && result->type == cJSON_Number)
        {
            modbustcp_master_info.port = result->valueint;
        }
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Get_Modbus_Tcp_Master_List(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    // // 遍历数组元素
    // for (auto& info : modbustcp_master_list.master_list) {
    //     info.master_name.clear();  
    //     info.ip.clear();
    //     info.port = 0;
    // }
    modbustcp_master_list = rm_msgs::Modbustcpmasterlist();
    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    result = cJSON_GetObjectItem(root, "list");
    if (result != NULL && result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        for (int i = 0; i < size; i++) {
            cJSON* master_item = cJSON_GetArrayItem(result, i);
            rm_msgs::Modbustcpmasterinfo info;
            cJSON* ip_JSON = cJSON_GetObjectItem(master_item, "ip");
            cJSON* mn_JSON = cJSON_GetObjectItem(master_item, "master_name");
            cJSON* port_JSON = cJSON_GetObjectItem(master_item, "port");
            if(ip_JSON!=NULL && ip_JSON->type == cJSON_String)
                info.ip = ip_JSON->valuestring;
            if(mn_JSON!=NULL && mn_JSON->type == cJSON_String)
                info.master_name = mn_JSON->valuestring;
            if(port_JSON!=NULL && port_JSON->type == cJSON_Number)
                info.port = port_JSON->valueint;
            modbustcp_master_list.master_list.push_back(info);
        }
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}
int Parser_Controller_Rs485_Mode_V4(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    result = cJSON_GetObjectItem(root, "controller_rs485_mode");
    if (result != NULL && result->type == cJSON_Number)
    {
        get_modbus_mode.mode = result->valueint;
        result = cJSON_GetObjectItem(root, "baudrate");
        if(result!=NULL && result->type == cJSON_Number)
            get_modbus_mode.baudrate = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}
int Parser_Tool_Rs485_Mode_V4(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    result = cJSON_GetObjectItem(root, "tool_rs485_mode");
    if (result != NULL && result->type == cJSON_Number)
    {
        get_modbus_mode.mode = result->valueint;
        result = cJSON_GetObjectItem(root, "baudrate");
        if(result!=NULL && result->type == cJSON_Number)
            get_modbus_mode.baudrate = result->valueint;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Modbus_Data(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    read_modbus_data.data.clear();
    if (root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }
    result = cJSON_GetObjectItem(root, "data");
    if (result != NULL && result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        for (int i = 0; i < size; i++) {
            cJSON* data_item = cJSON_GetArrayItem(result, i);
            if(data_item!=NULL && data_item->type == cJSON_Number)
                read_modbus_data.data.push_back(data_item->valueint);
        }
        read_modbus_data.state = true;
        cJSON_Delete(root);
        return 0;
    }
    else
        return 3;
}

int Parser_Msg(char *msg)
{
    cJSON *root = NULL, *json_state, *json_sub;
    root = cJSON_Parse(msg);
    // ROS_INFO("all msg data is:%s", msg);
    int res = 0;
    uint32_t plan_version;
    std::string data;
    std::stringstream ss;
    char buffer[20];

    if (root == NULL)
    {
        cJSON_Delete(root);
        return -1;
    }

    // Update:2023-7-25 @HermanYe
    // Get controller version
    json_state = cJSON_GetObjectItem(root, "Product_version");
    if(json_state != NULL)
    { 
    /***********************************获得机械臂型号****************************************/  
        data = json_state->valuestring;
        std::cout << "Arm type is " << data << std::endl;
        RM_Joint.product_version = data;
        int data_len = data.length();
        arm_dof = 6;
        force_sensor = 12;
        for(int i=0;i<data_len;i++)
        {
            if(data[i]=='7')
            {
                arm_dof = 7;
            }
            if(data[i]=='F')
            {
                force_sensor = 1;
            }

        }
    /***********************************获得软件版本型号****************************************/  
        // json_state = cJSON_GetObjectItem(root, "Plan_version");
        // plan_version = json_state->valueint;
        // ss << std::hex << std::setw(2) << std::setfill('0') << (int)plan_version;
        // // std::string hex_str = 
        // // hex_str = toupper(hex_str[1]);
        // RM_Joint.plan_version = ss.str();
        // strcpy(buffer, RM_Joint.plan_version.c_str());
        //std::cout << "Arm version is " << buffer << std::endl;
    /**************************************区分六维力版本****************************************/  
        // if(buffer[1] == 'b')
        // {
        //     force_sensor = 12;
        // }
        // else if(buffer[1] == 'f')
        // {
        //     force_sensor = 1;
        // }
        // else if(buffer[1] == 'd')
        // {
        //     force_sensor = 2;
        // }
        // if(buffer[0] == '6')
        // {
        //     arm_dof = 6; 
        // }
        // else if(buffer[0] == '7')
        // {
        //     arm_dof = 7;
        // }
        // 之前控制器版本不支持UDP反馈
        json_state = cJSON_GetObjectItem(root, "Product_version");
        if(json_state != NULL) {
            CONTROLLER_VERSION = 2;
            res = Parser_Arm_Software_Info(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return CTRL_VERSION;
            }
            else
            {
                cJSON_Delete(root);
                return -10;
            }
            // 2为带UDP反馈版本控制器

        } else {
            CONTROLLER_VERSION = 1;
            // 1为不带UDP反馈版本控制器
            // 0为版本号解析失败
        }
        json_state = cJSON_GetObjectItem(root, "robot_controller_version");
        if (json_state != NULL)
        {
            CONTROLLER_VERSION = 2;
            res = Parser_Arm_Software_Info(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return CTRL_VERSION;
            }
            else
            {
                cJSON_Delete(root);
                return -10;
            }
        }
    }    
    
    json_state = cJSON_GetObjectItem(root, "joint");
    if (json_state != NULL)
    {
        json_state = cJSON_GetObjectItem(root, "state");
        if (json_state != NULL && json_state->type == cJSON_String && (!strcmp("joint_degree", json_state->valuestring) || !strcmp("joint_state", json_state->valuestring)))
        {   
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
        if (json_state->type == cJSON_String && !strcmp("expand_state", json_state->valuestring))
        {
            res = Parser_Expand_State(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                return EXPAND_CURRENT_STATE;
            }
            else
            {
                cJSON_Delete(root);
                return -3;
            }
        }
        if (json_state->type == cJSON_String && !strcmp("tool_IO_state", json_state->valuestring))
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
                return -3;
            }
        }
        if (json_state->type == cJSON_String && !strcmp("joint_software_version", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "version");
            if (json_state != NULL && json_state->type == cJSON_Array)
            {
                int size = cJSON_GetArraySize(json_state);
                //std::cout << "cJSON_GetArraySize:" << size << std::endl;
                for (int i = 0; i < size; i++)
                {
                    cJSON *json_sub = cJSON_GetArrayItem(json_state, i);
                    if(json_sub!=NULL && json_sub->type == cJSON_String)
                        joint_software_version.joint_version[i] = json_sub->valuestring;
                        //std::cout << "joint_software_version.joint_version[0]: " << joint_software_version.joint_version[i] << std::endl;
                }
                return JOINT_VERSION;
            }
            else
            {
                cJSON_Delete(root);
                return -10;
            }
        }
        if (json_state->type == cJSON_String && !strcmp("tool_software_version", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "version");
            if (json_state != NULL && json_state->type == cJSON_String){
                tool_software_version.data = json_state->valuestring;
                return TOOL_VERSION;
            }
            else
            {
                cJSON_Delete(root);
                return -10;
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

    json_state = cJSON_GetObjectItem(root, "DI");
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
            if(CONTROLLER_VERSION == 1)
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
            else
            {
                return -5;
            }
        }
    }

    json_state = cJSON_GetObjectItem(root, "trajectory_state");
    if (json_state != NULL)
    {
        // ROS_INFO("33333333333333333333");
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
    
    json_state = cJSON_GetObjectItem(root, "device");
    if(json_state!=NULL)
    {
        json_state = cJSON_GetObjectItem(root, "state");
        if (json_state != NULL && json_state->type == cJSON_String && (!strcmp("current_trajectory_state", json_state->valuestring)))
        {   
            res = Parser_Lift_InPosition(msg);
            if (res == 0)
            {
                cJSON_Delete(root);
                if (lift_in_position.device == 3)
                {
                    return LIFT_IN_POSITION;
                }else if(lift_in_position.device == 4){
                    expand_in_position.device = lift_in_position.device;
                    expand_in_position.state = lift_in_position.state;
                    expand_in_position.trajectory_connect = lift_in_position.trajectory_connect;
                    expand_in_position.trajectory_state = lift_in_position.trajectory_state;
                    return EXPAND_IN_POSITION;
                }
            }
            else
            {
                cJSON_Delete(root);
                return -5;
            }
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
        else if (!strcmp("set_rm_plus_mode", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_RM_PLUS_MODE;
                }
                else if (json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_RM_PLUS_MODE;
                }
            }
        }
        else if (!strcmp("get_rm_plus_mode", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "mode");
            if (json_state != NULL)
            {
                RM_Joint.mode_result = json_state->valueint;
                return GET_RM_PLUS_MODE;
            }
        }
        else if (!strcmp("set_rm_plus_touch", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_RM_PLUS_TOUCH;
                }
                else if (json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_RM_PLUS_TOUCH;
                }
            }
        }
        else if (!strcmp("get_rm_plus_touch", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "mode");
            if (json_state != NULL)
            {
                RM_Joint.mode_result = json_state->valueint;
                return GET_RM_PLUS_TOUCH;
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
        else if (!strcmp("set_force_position", json_state->valuestring))
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
        else if (!strcmp("get_Fz", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "Fz");
            if (json_state != NULL)
            {
                res = Parser_One_Force_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return GET_ONE_FORCE;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if (!strcmp("stop_force_position", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "stop_state");
            if (json_state != NULL)
            {
                res = Parser_Stop_Force_Position(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return STOP_FORCE_POSITION;
                }
                else
                {
                    cJSON_Delete(root);
                    ROS_INFO("Parser stop_force_position failed");
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
        /**************************示教返回******************************/
        else if(!strcmp("set_joint_teach", json_state->valuestring))      //关节示教
        {
            json_state = cJSON_GetObjectItem(root, "joint_teach");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_JOINT_TEACH;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_JOINT_TEACH;
                }
            }
        }
        else if(!strcmp("set_pos_teach", json_state->valuestring))      //位置示教
        {
            json_state = cJSON_GetObjectItem(root, "pos_teach");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_POS_TEACH;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_POS_TEACH;
                }
            }
        }
        else if(!strcmp("set_ort_teach", json_state->valuestring))      //姿态示教
        {
            json_state = cJSON_GetObjectItem(root, "ort_teach");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_ORT_TEACH;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_ORT_TEACH;
                }
            }
        }
        /**************************示教返回******************************/
        else if(!strcmp("set_stop_teach", json_state->valuestring))      //示教停止
        {
            json_state = cJSON_GetObjectItem(root, "stop_teach");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_STOP_TEACH;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_STOP_TEACH;
                }
            }
        }
        else if(!strcmp("set_lift_speed", json_state->valuestring))  //升降速度控制返回
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_LIFT_SPEED;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_LIFT_SPEED;
                }
            } 
        }
        else if(!strcmp("expand_set_speed", json_state->valuestring))  //扩展关节速度控制返回
        {
           json_state = cJSON_GetObjectItem(root, "set_speed_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_EXPAND_SPEED;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_EXPAND_SPEED;
                }
            } 
        }
        else if(!strcmp("set_realtime_push", json_state->valuestring))//UDP端口控制返回
        {
           json_state = cJSON_GetObjectItem(root, "state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_REALTIME_PUSH;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_REALTIME_PUSH;
                }
            } 
        }
        else if(!strcmp("get_realtime_push", json_state->valuestring)) //UDP端口获取返回
        {
            json_state = cJSON_GetObjectItem(root, "cycle");
            if (json_state != NULL)
            {
                res = Parser_Get_Realtime_Push_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return GET_REALTIME_PUSH;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        /********************************灵巧手控制返回************************************/
        else if(!strcmp("set_hand_posture", json_state->valuestring))//手势返回
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_HAND_POSTURE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_HAND_POSTURE;
                }
            } 
        }
        else if(!strcmp("set_hand_seq", json_state->valuestring))//序列返回
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_HAND_SEQ;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_HAND_SEQ;
                }
            } 
        }
        else if(!strcmp("set_hand_angle", json_state->valuestring))//设置角度返回
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_HAND_ANGLE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_HAND_ANGLE;
                }
            } 
        }
        else if(!strcmp("set_hand_speed", json_state->valuestring))//设置速度返回
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_HAND_SPEED;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_HAND_SPEED;
                }
            } 
        }
        else if(!strcmp("set_hand_force", json_state->valuestring))//设置阈值返回
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_HAND_FORCE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_HAND_FORCE;
                }
            } 
        }
        else if(!strcmp("hand_follow_angle", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return HAND_FOLLOW_ANGLE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return HAND_FOLLOW_ANGLE;
                }
            } 
        }
        else if(!strcmp("hand_follow_pos", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return HAND_FOLLOW_POS;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return HAND_FOLLOW_POS;
                }
            } 
        }
        else if(!strcmp("set_arm_power", json_state->valuestring))//设置阈值返回
        {
           json_state = cJSON_GetObjectItem(root, "arm_power");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_ARM_POWER;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_ARM_POWER;
                }
            } 
        }
        else if(!strcmp("set_tool_voltage", json_state->valuestring))//设置电压返回
        {
           json_state = cJSON_GetObjectItem(root, "state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_TOOL_VOLTAGE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_TOOL_VOLTAGE;
                }
            } 
        }
        else if(!strcmp("set_tool_DO_state", json_state->valuestring))  //设置数字IO输出
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_TOOL_DO_STATE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_TOOL_DO_STATE;
                }
            } 
        }
        else if(!strcmp("set_DO_state", json_state->valuestring))  //设置数字IO输出
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_DO_STATE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_DO_STATE;
                }
            } 
        }
        else if(!strcmp("set_AO_state", json_state->valuestring))  //设置数字IO输出
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_AO_STATE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_AO_STATE;
                }
            } 
        }
        else if(!strcmp("set_arm_stop", json_state->valuestring))  //设置数字IO输出
        {
           json_state = cJSON_GetObjectItem(root, "arm_stop");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_ARM_STOP;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_ARM_STOP;
                }
            } 
        }
        else if(!strcmp("set_joint_clear_err", json_state->valuestring))  //设置数字IO输出
        {
           json_state = cJSON_GetObjectItem(root, "joint_clear_err");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_JOINT_CLEAR_ERR;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_JOINT_CLEAR_ERR;
                }
            } 
        }
        else if(!strcmp("set_joint_en_state", json_state->valuestring))  //设置数字IO输出
        {
           json_state = cJSON_GetObjectItem(root, "joint_en_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_JOINT_EN_STATE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_JOINT_EN_STATE;
                }
            } 
        }
        else if(!strcmp("clear_system_err", json_state->valuestring))  //设置数字IO输出
        {
           json_state = cJSON_GetObjectItem(root, "clear_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_SYSTEM_EN_STATE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_SYSTEM_EN_STATE;
                }
            } 
        }
        else if(!strcmp("set_arm_continue", json_state->valuestring))//设置速度返回
        {
           json_state = cJSON_GetObjectItem(root, "arm_continue");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_ARM_CONTINUE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_ARM_CONTINUE;
                }
            } 
        }
        else if(!strcmp("set_arm_pause", json_state->valuestring))//设置速度返回
        {
           json_state = cJSON_GetObjectItem(root, "arm_pause");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_ARM_PAUSE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_ARM_PAUSE;
                }
            } 
        }
        /********************************升降机构返回************************************/
        else if(!strcmp("set_lift_height", json_state->valuestring))//位置闭环控制设置输出
        {
           json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.plan_flag = 1;
                    return PLAN_STATE_TYPE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.plan_flag = 0;
                    return PLAN_STATE_TYPE;
                }
            } 
        }
        /********************************扩展关节返回************************************/
        else if(!strcmp("expand_set_pos", json_state->valuestring))//位置闭环控制设置输出
        {
           json_state = cJSON_GetObjectItem(root, "set_pos_state");
            if (json_state != NULL)
            {
                if (json_state->type == cJSON_True)
                {
                    RM_Joint.plan_flag = 1;
                    return PLAN_STATE_TYPE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.plan_flag = 0;
                    return PLAN_STATE_TYPE;
                }
            } 
        }
        /********************************Modbus模式配置************************************/
        else if(!strcmp("get_controller_RS485_mode", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root,"baudrate");
            if(json_state!=NULL && json_state->type == cJSON_Number)
            {
                modbus_data.get_controller_RS485_mode.baudrate = json_state->valueint;
            }
            json_state = cJSON_GetObjectItem(root,"controller_RS485_mode");
            if(json_state!=NULL && json_state->type == cJSON_Number)
            {
                modbus_data.get_controller_RS485_mode.controller_RS485_mode = json_state->valueint;
            }
            json_state = cJSON_GetObjectItem(root,"modbus_timeout");
            if(json_state!=NULL && json_state->type == cJSON_Number)
            {
                modbus_data.get_controller_RS485_mode.modbus_timeout = json_state->valueint;
            }
            cJSON_Delete(root);
            return GET_CONTROLLER_RS485_MODE;
        }
        else if(!strcmp("get_tool_RS485_mode", json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root,"baudrate");
            if(json_state!=NULL && json_state->type == cJSON_Number)
            {
                modbus_data.get_tool_RS485_mode.baudrate = json_state->valueint;
            }
            json_state = cJSON_GetObjectItem(root,"tool_RS485_mode");
            if(json_state!=NULL && json_state->type == cJSON_Number)
            {
                modbus_data.get_tool_RS485_mode.tool_RS485_mode = json_state->valueint;
            }
            json_state = cJSON_GetObjectItem(root,"modbus_timeout");
            if(json_state!=NULL && json_state->type == cJSON_Number)
            {
                modbus_data.get_tool_RS485_mode.modbus_timeout = json_state->valueint;
            }
            cJSON_Delete(root);
            return GET_TOOL_RS485_MODE;
        }
        else if(!strcmp("set_modbus_mode",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    modbus_data.state.data = true;
                    return SET_MODBUS_MODE;
                }
                else if(json_state->type == cJSON_False)
                {
                    modbus_data.state.data = false;
                    return SET_MODBUS_MODE;
                }
            }
        }
        else if(!strcmp("close_modbus_mode",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    modbus_data.state.data = true;
                    return CLOSE_MODBUS_MODE;
                }
                else if(json_state->type == cJSON_False)
                {
                    modbus_data.state.data = false;
                    return CLOSE_MODBUS_MODE;
                }
            }
        }
        else if(!strcmp("set_modbustcp_mode",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    modbus_data.state.data = true;
                    return SET_MODBUSTCP_MODE;
                }
                else if(json_state->type == cJSON_False)
                {
                    modbus_data.state.data = false;
                    return SET_MODBUSTCP_MODE;
                }
            }
        }
        else if(!strcmp("close_modbustcp_mode",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    modbus_data.state.data = true;
                    return CLOSE_MODBUSTCP_MODE;
                }
                else if(json_state->type == cJSON_False)
                {
                    modbus_data.state.data = false;
                    return CLOSE_MODBUSTCP_MODE;
                }
            }
        }
        /********************************Modbus读写寄存器************************************/
        else if(!strcmp("read_coils",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "data");
            if(json_state != NULL && json_state->type == cJSON_Number)
            {
                modbus_data.read_coils.data[0] = json_state->valueint;
                modbus_data.read_coils.state = true;
            }
            json_state = cJSON_GetObjectItem(root, "read_state");
            if(json_state != NULL && json_state->type == cJSON_False)
            {
                modbus_data.read_coils.state = false;
                return READ_COILS;
            }
            return READ_COILS;
        }
        else if(!strcmp("read_multiple_coils",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "data");
            if(json_state != NULL && json_state->type == cJSON_Array)
            {
                int size = cJSON_GetArraySize(json_state);
                modbus_data.read_multiple_coils.data.resize(size);
                for (int i=0;i<size;i++)
                {
                    json_sub = cJSON_GetArrayItem(json_state,i);
                    modbus_data.read_multiple_coils.data[i]=json_sub->valueint;
                }
                modbus_data.read_multiple_coils.state = true;
            }
            json_state = cJSON_GetObjectItem(root, "read_state");
            if(json_state != NULL && json_state->type == cJSON_False)
            {
                modbus_data.read_multiple_coils.state = false;
                return READ_MULTIPLE_COILS;
            }
            return READ_MULTIPLE_COILS;
        }
        else if(!strcmp("read_input_status",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "data");
            if(json_state != NULL && json_state->type == cJSON_Number)
            {
                modbus_data.read_input_status.data[0] = json_state->valueint;
                modbus_data.read_input_status.state = true;
            }
            json_state = cJSON_GetObjectItem(root, "read_state");
            if(json_state != NULL && json_state->type == cJSON_False)
            {
                modbus_data.read_input_status.state = false;
                return READ_INPUT_STATUS;
            }
            return READ_INPUT_STATUS;
        }
        else if(!strcmp("read_holding_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "data");
            if(json_state != NULL && json_state->type == cJSON_Number)
            {
                modbus_data.read_holding_registers.data[0] = json_state->valueint;
                modbus_data.read_holding_registers.state = true;
            }
            json_state = cJSON_GetObjectItem(root, "read_state");
            if(json_state != NULL && json_state->type == cJSON_False)
            {
                modbus_data.read_holding_registers.state = false;
                return READ_HOLDING_REGISTERS;
            }
            return READ_HOLDING_REGISTERS;
        }
        else if(!strcmp("read_multiple_holding_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "data");
            if(json_state != NULL && json_state->type == cJSON_Array)
            {
                int size = cJSON_GetArraySize(json_state);
                modbus_data.read_multiple_holding_registers.data.resize(size);
                for (int i=0;i<size;i++)
                {
                    json_sub = cJSON_GetArrayItem(json_state,i);
                    modbus_data.read_multiple_holding_registers.data[i]=json_sub->valueint;
                }
                modbus_data.read_multiple_holding_registers.state = true;
            }
            json_state = cJSON_GetObjectItem(root, "read_state");
            if(json_state != NULL && json_state->type == cJSON_False)
            {
                modbus_data.read_multiple_holding_registers.state = false;
                return READ_MULTIPLE_HOLDING_REGISTERS;
            }
            return READ_MULTIPLE_HOLDING_REGISTERS;
        }
        else if(!strcmp("read_input_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "data");
            if(json_state != NULL && json_state->type == cJSON_Number)
            {
                modbus_data.read_input_registers.data[0] = json_state->valueint;
                modbus_data.read_input_registers.state = true;
            }
            json_state = cJSON_GetObjectItem(root, "read_state");
            if(json_state != NULL && json_state->type == cJSON_False)
            {
                modbus_data.read_input_registers.state = false;
                return READ_INPUT_REGISTERS;
            }
            return READ_INPUT_REGISTERS;
        }
        else if(!strcmp("read_multiple_input_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "data");
            
            if(json_state != NULL && json_state->type == cJSON_Array)
            {
                int size = cJSON_GetArraySize(json_state);
                modbus_data.read_multiple_input_registers.data.resize(size);
                for (int i=0;i<size;i++)
                {
                    json_sub = cJSON_GetArrayItem(json_state,i);
                    modbus_data.read_multiple_input_registers.data[i]=json_sub->valueint;
                }
                modbus_data.read_multiple_input_registers.state = true;
            }
            json_state = cJSON_GetObjectItem(root, "read_state");
            if(json_state != NULL && json_state->type == cJSON_False)
            {
                modbus_data.read_multiple_input_registers.state = false;
                return READ_MULTIPLE_INPUT_REGISTERS;
            }
            return READ_MULTIPLE_INPUT_REGISTERS;
        }
        else if(!strcmp("write_single_coil",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    modbus_data.state.data = true;
                    return WRITE_SINGLE_COIL;
                }
                else if(json_state->type == cJSON_False)
                {
                    modbus_data.state.data = false;
                    return WRITE_SINGLE_COIL;
                }
            }
        }
        else if(!strcmp("write_coils",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    modbus_data.state.data = true;
                    return WRITE_COILS;
                }
                else if(json_state->type == cJSON_False)
                {
                    modbus_data.state.data = false;
                    return WRITE_COILS;
                }
            }
        }
        else if(!strcmp("write_single_register",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    modbus_data.state.data = true;
                    return WRITE_SINGLE_REGISTER;
                }
                else if(json_state->type == cJSON_False)
                {
                    modbus_data.state.data = false;
                    return WRITE_SINGLE_REGISTER;
                }
            }
        }
        else if(!strcmp("write_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    modbus_data.state.data = true;
                    return WRITE_REGISTERS;
                }
                else if(json_state->type == cJSON_False)
                {
                    modbus_data.state.data = false;
                    return WRITE_REGISTERS;
                }
            }
        }
        else if(!strcmp("set_arm_emergency_stop",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_ARM_EMERGENCY_STOP;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_ARM_EMERGENCY_STOP;
                }
            }
        }
        else if(!strcmp("get_trajectory_file_list",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "page_num");
            if (json_state != NULL)
            {
                res = Parser_Trajectory_List(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return GET_TRAJECTORY_FILE_LIST;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("set_run_trajectory_file",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "run_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_RUN_TRAJECTORY;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_RUN_TRAJECTORY;
                }
            }
        }
        else if(!strcmp("delete_trajectory_file",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "delete_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return DELETE_TRAJECTORY_FILE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return DELETE_TRAJECTORY_FILE;
                }
            }
        }
        else if(!strcmp("save_trajectory_file",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "save_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SAVE_TRAJECTORY_FILE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SAVE_TRAJECTORY_FILE;
                }
            }
        }
        else if(!strcmp("get_flowchart_program_run_state",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "id");
            if (json_state != NULL)
            {
                res = Parser_Flowchart_Runstate(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return GET_FLOWCHART_PROGRAM_RUN_STATE;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("movel_offset",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "receive_state");
            if(json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return MOVEL_OFFSET;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return MOVEL_OFFSET;
                }
            }
        }
        else if(!strcmp("arm_software_info",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "robot_controller_version");
            if (json_state != NULL)
            {
                res = Parser_Arm_Software_Info(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return ARM_VERSION;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("add_modbus_tcp_master",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "add_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return ADD_MODBUS_TCP_MASTER;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return ADD_MODBUS_TCP_MASTER;
                }
            }
        }
        else if(!strcmp("update_modbus_tcp_master",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "update_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return UPDATE_MODBUS_TCP_MASTER;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return UPDATE_MODBUS_TCP_MASTER;
                }
            }
        }
        else if(!strcmp("delete_modbus_tcp_master",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "delete_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return DELETE_MODBUS_TCP_MASTER;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return DELETE_MODBUS_TCP_MASTER;
                }
            }
        }
        else if(!strcmp("given_modbus_tcp_master",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "ip");
            if (json_state != NULL)
            {
                res = Parser_Get_Modbus_Tcp_Master(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return GET_MODBUS_TCP_MASTER;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("get_modbus_tcp_master_list",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "list");
            if (json_state != NULL)
            {
                res = Parser_Get_Modbus_Tcp_Master_List(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return GET_MODBUS_TCP_MASTER_LIST;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("set_controller_rs485_mode",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_CONTROLLER_RS485_MODE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_CONTROLLER_RS485_MODE;
                }
            }
        }
        else if(!strcmp("get_controller_rs485_mode",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "controller_rs485_mode");
            if (json_state != NULL)
            {
                res = Parser_Controller_Rs485_Mode_V4(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return GET_CONTROLLER_RS485_MODE_V4;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("set_tool_rs485_mode",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "set_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return SET_TOOL_RS485_MODE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return SET_TOOL_RS485_MODE;
                }
            }
        }
        else if(!strcmp("get_tool_rs485_mode",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "tool_rs485_mode");
            if (json_state != NULL)
            {
                res = Parser_Tool_Rs485_Mode_V4(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    return GET_TOOL_RS485_MODE_V4;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("read_modbus_tcp_coils",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "read_state");
            if (json_state != NULL&&json_state->type==cJSON_False)
            {
                read_modbus_data.state = false;
                read_modbus_data.data.clear();
                Modbus_Read_Case = 1;
                return MODBUS_READ;
                

            }
            json_state = cJSON_GetObjectItem(root, "data");
            if (json_state != NULL)
            {
                res = Parser_Modbus_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    Modbus_Read_Case = 1;
                    return MODBUS_READ;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("read_modbus_tcp_input_status",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "read_state");
            if (json_state != NULL&&json_state->type==cJSON_False)
            {
                read_modbus_data.state = false;
                read_modbus_data.data.clear();
                Modbus_Read_Case = 2;
                return MODBUS_READ;
            }
            json_state = cJSON_GetObjectItem(root, "data");
            if (json_state != NULL)
            {
                res = Parser_Modbus_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    Modbus_Read_Case = 2;
                    return MODBUS_READ;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("read_modbus_tcp_holding_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "read_state");
            if (json_state != NULL&&json_state->type==cJSON_False)
            {
                read_modbus_data.state = false;
                read_modbus_data.data.clear();
                Modbus_Read_Case = 3;
                return MODBUS_READ;
            }
            json_state = cJSON_GetObjectItem(root, "data");
            if (json_state != NULL)
            {
                res = Parser_Modbus_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    Modbus_Read_Case = 3;
                    return MODBUS_READ;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("read_modbus_tcp_input_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "read_state");
            if (json_state != NULL&&json_state->type==cJSON_False)
            {
                read_modbus_data.state = false;
                read_modbus_data.data.clear();
                Modbus_Read_Case = 4;
                return MODBUS_READ;
            }
            json_state = cJSON_GetObjectItem(root, "data");
            if (json_state != NULL)
            {
                res = Parser_Modbus_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    Modbus_Read_Case = 4;
                    return MODBUS_READ;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("write_modbus_rtu_coils",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    Modbus_Write_Case = 1;
                    return MODBUS_WRITE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    Modbus_Write_Case = 1;
                    return MODBUS_WRITE;
                    
                }
            }
        }
        else if(!strcmp("write_modbus_rtu_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    Modbus_Write_Case = 2;
                    return MODBUS_WRITE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    Modbus_Write_Case = 2;
                    return MODBUS_WRITE;
                    
                }
            }
        }
        else if(!strcmp("write_modbus_tcp_coils",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    Modbus_Write_Case = 1;
                    return MODBUS_WRITE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    Modbus_Write_Case = 1;
                    return MODBUS_WRITE;
                }
            }
        }
        else if(!strcmp("write_modbus_tcp_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    Modbus_Write_Case = 2;
                    return MODBUS_WRITE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    Modbus_Write_Case = 2;
                    return MODBUS_WRITE;
                }
            }
        }
        // -----------------------------------------
        else if(!strcmp("read_modbus_rtu_coils",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "read_state");
            if (json_state != NULL&&json_state->type==cJSON_False)
            {
                read_modbus_data.state = false;
                read_modbus_data.data.clear();
                Modbus_Read_Case = 1;
                return MODBUS_READ;
            }
            json_state = cJSON_GetObjectItem(root, "data");
            if (json_state != NULL)
            {
                res = Parser_Modbus_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    Modbus_Read_Case = 1;
                    return MODBUS_READ;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("read_modbus_rtu_input_status",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "read_state");
            if (json_state != NULL&&json_state->type==cJSON_False)
            {
                read_modbus_data.state = false;
                read_modbus_data.data.clear();
                Modbus_Read_Case = 2;
                return MODBUS_READ;
            }
            json_state = cJSON_GetObjectItem(root, "data");
            if (json_state != NULL)
            {
                res = Parser_Modbus_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    Modbus_Read_Case = 2;
                    return MODBUS_READ;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("read_modbus_rtu_holding_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "read_state");
            if (json_state != NULL&&json_state->type==cJSON_False)
            {
                read_modbus_data.state = false;
                read_modbus_data.data.clear();
                Modbus_Read_Case = 3;
                return MODBUS_READ;
            }
            json_state = cJSON_GetObjectItem(root, "data");
            if (json_state != NULL)
            {
                res = Parser_Modbus_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    Modbus_Read_Case = 3;
                    return MODBUS_READ;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("read_modbus_rtu_input_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "read_state");
            if (json_state != NULL&&json_state->type==cJSON_False)
            {
                read_modbus_data.state = false;
                read_modbus_data.data.clear();
                Modbus_Read_Case = 4;
                return MODBUS_READ;
            }
            json_state = cJSON_GetObjectItem(root, "data");
            if (json_state != NULL)
            {
                res = Parser_Modbus_Data(msg);
                if (res == 0)
                {
                    cJSON_Delete(root);
                    Modbus_Read_Case = 4;
                    return MODBUS_READ;
                }
                else
                {
                    cJSON_Delete(root);
                    return -10;
                }
            }
        }
        else if(!strcmp("write_modbus_rtu_coils",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return MODBUS_WRITE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return MODBUS_WRITE;
                }
            }
        }
        else if(!strcmp("write_modbus_rtu_registers",json_state->valuestring))
        {
            json_state = cJSON_GetObjectItem(root, "write_state");
            if (json_state != NULL)
            {
                if(json_state->type == cJSON_True)
                {
                    RM_Joint.state = true;
                    return MODBUS_WRITE;
                }
                else if(json_state->type == cJSON_False)
                {
                    RM_Joint.state = false;
                    return MODBUS_WRITE;
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
            ROS_ERROR("Joint %d target position over limit err!\n", i + 1);
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
    case ARM_ERR_OVERSPEED_LIMIT:
        ROS_ERROR("arm error: speed limit!\n");
        break;
    case ARM_ERR_ACCELERATION_LIMIT:
        ROS_ERROR("arm error: acceleration limit!\n");
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

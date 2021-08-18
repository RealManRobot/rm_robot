#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
//messgae
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
#include <rm_msgs/Tool_Analog_Output.h>
#include <rm_msgs/Tool_Digital_Output.h>
#include <rm_msgs/Tool_IO_State.h>
#include <rm_msgs/Plan_State.h>
#include <rm_msgs/Stop.h>
#include <rm_msgs/IO_Update.h>


#ifdef __cplusplus
extern "C"{
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
//Copyright(C) 睿尔曼智能科技有限公司
//All rights reserved
//文档说明：该文档定义了机械臂接口函数的实现方式
//////////////////////////////////////////////////////////////////////////////////
#define RAD_DEGREE       57.2958
#define DEGREE_RAD     0.01745


//Gripper open width
#define  GRIPPER_WIDTH     0.07       //Unit:m
#define  GRIPPER_SCALE     1000

//系统初始化错误代码
#define   SYS_OK               0x0000         //系统运行正常
#define   TEMP_SENSOR_ERR      0x0001         //数字温度传感器初始化错误
#define   POS_SENSOR_ERR       0x0002         //九轴数字传感器初始化错误
#define   SD_CARD_ERR          0x0003         //SD卡初始化错误
#define   NAND_FLASH_ERR       0x0004         //NAND FLASH初始化错误
#define   BLUE_TEETH_ERR       0x0005         //蓝牙设备初始化错误
#define   RM58S_INIT_ERR       0x0006         //RM58SWIFI模块初始化错误
#define   CTRL_SYS_COM_ERR     0x0007         //实时层系统未按时上传数据
#define   CTRL_SYS_INIT_ERR    0x0008         //实时层系统初始化错误
#define   STATE_QUEUE_INIT_ERR 0x0009         //上传队列初始化错误
#define   ZLAN1003_INIT_ERR    0x0010         //ZLAN1003初始化错误
#define   TEMPERATURE_INIT_ERR 0x0011         //温度传感器初始化错误
#define   DA_SENSOR_INIT_ERR   0x0012         //DA芯片初始化错误


//系统运行中DA错误
#define ARM_POSE_FALL         0x5001      //机械臂底座倾倒
#define POSE_SENSOR_ERR       0x5002      //九轴数据错误
#define SYS_TEMPER_ERR        0x5003      //控制器过温
#define TEMPER_SENSOR_ERR     0x5004      //温度传感器数据错误
#define SYS_CURRENT_OVELOAD   0x5005      //控制器过流
#define SYS_CURRENT_UNDER     0x5006      //控制器欠流
#define SYS_VOLTAGE_OVELOAD   0x5007      //控制器过压
#define SYS_VOLTAGE_UNDER     0x5008      //控制器欠压
#define CTRL_SYS_LOSS_ERR     0x5009      //实时层无法通信

//实时层错误
#define COMM_ERR               0x1001         //通讯2S中断
#define JOINT_LIMIT_ERR        0x1002         //目标角度超过关节限位
#define INVERSE_KM_ERR         0x1003         //运动学逆解错误
#define M4_CTRL_ERR            0x1004         //M4内核通信错误
#define CAN_INIT_ERR           0x1005         //CAN外设初始化失败
#define APP_BOARD_LOSS         0x1006         //接口板无法通信
#define QUEUE_INIT_ERR         0x4001        //轨迹规划点队列无法创建

//机械臂关节错误类型
#define  ERR_MASK_OK             0X0000  //Normal state
#define  ERR_MASK_OVER_CURRENT   0X0001  //电机电流超过安全范围
#define  ERR_MASK_OVER_VOLTAGE   0X0002  //系统电压超过安全范围
#define  ERR_MASK_UNDER_VOLTAGE  0X0004  //系统电压低于安全范围
#define  ERR_MASK_OVER_TEMP      0X0008  //温度过高
#define  ERR_MASK_HALL           0X0010  //霍尔错误
#define  ERR_MASK_ENC            0X0020  //编码器出错
#define  ERR_MASK_POS_TRACK      0X0040  //位置误差跟踪超限保护
#define  ERR_MASK_CUR_ON         0X0080  //上电时电流传感器检测错误
#define  ERR_MASK_TEMP           0X0100  //温度传感器出错标志
#define  ERR_MASK_TAG_POS        0X0200  // 目标位置超限
#define  ERR_MASK_DRV8320        0x0400  // DRV8320错误
#define  JOINT_CAN_LOSE_ERR      0x0800          // 关节丢帧

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
}POSE;

//机械臂状态参数
typedef struct
{
    float joint[6];         //关节角度
    uint16_t err_flag[6];   //关节错误代码
    bool Arm_DI[3];
    float Arm_AI[4];
    bool Tool_DI[2];
    float Tool_AI;
    bool plan_flag;
    float gripper_joint;    //gripper
}JOINT_STATE;
JOINT_STATE RM_Joint;


//subscriber
ros::Subscriber MoveJ_Cmd, MoveL_Cmd, MoveC_Cmd, JointPos_Cmd, Arm_DO_Cmd, Arm_AO_Cmd, Tool_DO_Cmd, Tool_AO_Cmd, Gripper_Cmd, Gripper_Set_Cmd, Emergency_Stop, Joint_En, IO_Update;

//publisher
ros::Publisher Joint_State, Arm_IO_State, Tool_IO_State, Plan_State;

//timer
ros::Timer State_Timer;
int timer_cnt = 0;

#define ARM_JOINT_STATE    0x01
#define ARM_JOINT_ERR      0x02
#define ARM_IO_INPUT       0x03
#define TOOL_IO_INPUT      0x04
#define PLAN_STATE_TYPE    0x05

int Arm_Socket;                //机械臂网络通信套接字
const char *Arm_IP = "192.168.1.18";   //机械臂IP地址
int Arm_Port = 8080;              //机械臂端口地址

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

    while(1)
    {
        nRet = select(FD_SETSIZE, &fds, NULL, NULL, &time_out);
        if(nRet == 0)
            break;
        recv(Arm_Socket, temp, 1, 0);
    }
}

//获取机械臂关节错误代码
int Get_Joint_Err_Flag(void)
{
    cJSON *root;
    char* data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_joint_err_flag");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//关节空间规划
int Movej_Cmd(float *joint, byte v)
{
    cJSON *root, *array;
    char* data;
    char buffer[200];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    cJSON_AddNumberToObject(array, "test",  (int)(joint[0]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[1]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[2]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[3]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[4]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[5]*1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movej");
    cJSON_AddItemToObject(root, "joint", array);
    cJSON_AddNumberToObject(root, "v", v);
    cJSON_AddNumberToObject(root, "r", (int)(r*1000));

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}
//笛卡尔空间直线规划
int Movel_Cmd(POSE pose, byte v)
{
    cJSON *root, *array;
    char* data;
    char buffer[200];
    int res;
    int r = 0;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    //位置数据，0.001mm
    cJSON_AddNumberToObject(array, "test",  (int)(pose.px*1000000));
    cJSON_AddNumberToObject(array, "test",  (int)(pose.py*1000000));
    cJSON_AddNumberToObject(array, "test",  (int)(pose.pz*1000000));
    //姿态数据，0.001rad
    cJSON_AddNumberToObject(array, "test",  (int)(pose.rx*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(pose.ry*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(pose.rz*1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movel");
    cJSON_AddItemToObject(root, "pose", array);
    cJSON_AddNumberToObject(root, "v", v);
    cJSON_AddNumberToObject(root, "r", (int)(r*1000));

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//笛卡尔空间直线规划
int Movec_Cmd(POSE pose_via, POSE pose_to, byte v)
{
    cJSON *root, *array1, *array2, *pose_json;
    char* data;
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
    cJSON_AddNumberToObject(array1, "test",  (int)(pose_via.px*1000000));
    cJSON_AddNumberToObject(array1, "test",  (int)(pose_via.py*1000000));
    cJSON_AddNumberToObject(array1, "test",  (int)(pose_via.pz*1000000));
    //姿态数据，0.001rad
    cJSON_AddNumberToObject(array1, "test",  (int)(pose_via.rx*1000));
    cJSON_AddNumberToObject(array1, "test",  (int)(pose_via.ry*1000));
    cJSON_AddNumberToObject(array1, "test",  (int)(pose_via.rz*1000));

    //数组加入终点数据
    //位置数据，0.001mm
    cJSON_AddNumberToObject(array2, "test",  (int)(pose_to.px*1000000));
    cJSON_AddNumberToObject(array2, "test",  (int)(pose_to.py*1000000));
    cJSON_AddNumberToObject(array2, "test",  (int)(pose_to.pz*1000000));
    //姿态数据，0.001rad
    cJSON_AddNumberToObject(array2, "test",  (int)(pose_to.rx*1000));
    cJSON_AddNumberToObject(array2, "test",  (int)(pose_to.ry*1000));
    cJSON_AddNumberToObject(array2, "test",  (int)(pose_to.rz*1000));

    cJSON_AddItemToObject(pose_json, "pose_via", array1);
    cJSON_AddItemToObject(pose_json, "pose_to", array2);

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movec");
    cJSON_AddItemToObject(root, "pose", pose_json);
    cJSON_AddNumberToObject(root, "v", v);
    cJSON_AddNumberToObject(root, "r", (int)(r*1000));
    cJSON_AddNumberToObject(root, "loop", loop);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}
//角度透传
int Movej_CANFD(float *joint)
{
    cJSON *root, *array;
    char* data;
    char buffer[200];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();
    array = cJSON_CreateArray();
    //数组加入数据
    cJSON_AddNumberToObject(array, "test",  (int)(joint[0]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[1]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[2]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[3]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[4]*1000));
    cJSON_AddNumberToObject(array, "test",  (int)(joint[5]*1000));

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "movej_canfd");
    cJSON_AddItemToObject(root, "joint", array);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }

    return 0;
}
//急停指令
int Move_Stop_Cmd(void)
{
    cJSON *root;
    char* data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_arm_stop");

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//Set_DO_State:设置数字IO输出
int Set_DO_State(byte num, bool state)
{
    cJSON *root;
    char* data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_DO_state");
    cJSON_AddNumberToObject(root, "IO_Num", num);
    if(state)
    {
        cJSON_AddNumberToObject(root, "state", 1);
    }
    else
    {
        cJSON_AddNumberToObject(root, "state", 0);
    }

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//Set_AO_State:设置模拟IO输出
int Set_AO_State(byte num, float voltage)
{
    cJSON *root;
    char* data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_AO_state");
    cJSON_AddNumberToObject(root, "IO_Num", num);
    cJSON_AddNumberToObject(root, "voltage", (int)(voltage*1000));


    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//获取所有IO输入状态
int Get_IO_Input(void)
{
    cJSON *root;
    char* data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_IO_input");


    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//get robot joint
int Get_Arm_Joint(void)
{
    cJSON *root;
    char* data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_joint_degree");


    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//Set_Tool_DO_State:设置工具端数字IO输出
int Set_Tool_DO_State(byte num, bool state)
{
    cJSON *root;
    char* data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_tool_DO_state");
    cJSON_AddNumberToObject(root, "IO_Num", num);
    if(state)
    {
        cJSON_AddNumberToObject(root, "state", 1);
    }
    else
    {
        cJSON_AddNumberToObject(root, "state", 0);
    }

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//Set_Tool_AO_State:设置工具端模拟IO输出
int Set_Tool_AO_State(float voltage)
{
    cJSON *root;
    char* data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_tool_AO_state");
    cJSON_AddNumberToObject(root, "voltage", (int)(voltage*1000));


    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}
//获取工具端所有IO输入状态
int Get_Tool_IO_Input(void)
{
    cJSON *root;
    char* data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "get_tool_IO_input");


    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}
//设置手爪松开
int Set_Gripper_Release(int speed)
{
    cJSON *root;
    char* data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_gripper_release");
    cJSON_AddNumberToObject(root, "speed", speed);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}
//设置手爪力控夹取
int Set_Gripper_Pick(int speed, int force)
{
    cJSON *root;
    char* data;
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
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//设置手爪开口度
int Set_Gripper(int position)
{
    cJSON *root;
    char* data;
    char buffer[100];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_gripper_position");
    cJSON_AddNumberToObject(root, "position", position);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//set joint Enable state
int Set_Joint_Enable(int num, bool state)
{
    cJSON *root, *array;
    char* data;
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
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}

//clear joint err
int Clear_Joint_Err(int num)
{
    cJSON *root;
    char* data;
    char buffer[50];
    int res;
    //创建根节点对象
    root = cJSON_CreateObject();

    //加入字符串对象
    cJSON_AddStringToObject(root, "command", "set_joint_clear_err");
    cJSON_AddNumberToObject(root, "joint_clear_err", num);

    data = cJSON_Print(root);

    sprintf(buffer, "%s\r\n", data);
    res = send(Arm_Socket, buffer, strlen(buffer), 0);
    cJSON_Delete(root);
    free(data);

    if(res < 0)
    {
        return 1;
    }
    return 0;
}
/***************************************************解码函数************************************************************/
//Parser Arm Joint
int Parser_Arm_Joint(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    int data[6];
    int i = 0;

    if(root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "joint");
    if(result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if(size == 6)
        {
            for(i=0;i<size;i++)
            {
                json_sub = cJSON_GetArrayItem(result, i);
                data[i] = json_sub->valueint;
                RM_Joint.joint[i] = data[i];
                RM_Joint.joint[i] = RM_Joint.joint[i]/1000;
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

//解析机械臂关节错误代码
int Parser_Get_Joint_Err_Flag(char *msg)
{
    cJSON *root = NULL, *result, *json_sub;
    root = cJSON_Parse(msg);
    int data[6];
    int i = 0;

    if(root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "err_flag");
    if(result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        if(size == 6)
        {
            for(i=0;i<size;i++)
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

    if(root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    //获取数字输入
    result = cJSON_GetObjectItem(root, "DI");
    if(result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        for(i=0;i<size;i++)
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
    if(result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        for(i=0;i<size;i++)
        {
            json_sub = cJSON_GetArrayItem(result, i);
            RM_Joint.Arm_AI[i] = (float)(json_sub->valueint)/1000;
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

    if(root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    //获取数字输入
    result = cJSON_GetObjectItem(root, "DI");
    if(result->type == cJSON_Array)
    {
        int size = cJSON_GetArraySize(result);
        for(i=0;i<size;i++)
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
    if(result->type == cJSON_Number)
    {
        RM_Joint.Tool_AI = (float)(result->valueint)/1000;
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

    if(root == NULL)
    {
        cJSON_Delete(root);
        return 2;
    }

    result = cJSON_GetObjectItem(root, "trajectory_state");
    if((result->type == cJSON_True) || (result->type == cJSON_False))
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

int Parser_Msg(char *msg)
{
    cJSON *root = NULL, *json_state;
    root = cJSON_Parse(msg);
    int res = 0;

    if(root == NULL)
    {
        cJSON_Delete(root);
        return -1;
    }

    json_state = cJSON_GetObjectItem(root, "joint");
    if(json_state != NULL)
    {
        res = Parser_Arm_Joint(msg);
        if(res == 0)
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
    json_state = cJSON_GetObjectItem(root, "err_flag");
    if(json_state != NULL)
    {
        res = Parser_Get_Joint_Err_Flag(msg);
        if(res == 0)
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
    if(json_state != NULL)
    {
        //Arm IO Input
        if(json_state->type == cJSON_Array)
        {
            res = Parser_IO_Input(msg);
            if(res == 0)
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
        //Tool IO Input
        else
        {
            res = Parser_Tool_IO_Input(msg);
            if(res == 0)
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
    if(json_state != NULL)
    {
        res = Parser_Plan_State(msg);
        if(res == 0)
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

    cJSON_Delete(root);
    return 0;
}

//Quater to Euler
POSE Quater_To_Euler(geometry_msgs::Pose target)
{
    POSE msg;

    msg.px = target.position.x;
    msg.py = target.position.y;
    msg.pz = target.position.z;

    msg.rx = atan2(2*(target.orientation.w*target.orientation.x + target.orientation.y*target.orientation.z), (1 - 2*(target.orientation.x*target.orientation.x + target.orientation.y*target.orientation.y)));
    msg.ry = asin(2*(target.orientation.w*target.orientation.y - target.orientation.z*target.orientation.x));
    msg.rz = atan2(2*(target.orientation.w*target.orientation.z + target.orientation.x*target.orientation.y), (1 - 2*(target.orientation.y*target.orientation.y + target.orientation.z*target.orientation.z)));
}

//处理关节错误
int Info_Joint_Err(void)
{
    int i = 0;

    for(i=0;i<6;i++)
    {
        switch(RM_Joint.err_flag[i])
        {
            case ERR_MASK_OK:
                break;
            case ERR_MASK_OVER_CURRENT:
                ROS_ERROR("Joint %d over current err!\n", i+1);
                break;
            case ERR_MASK_OVER_VOLTAGE:
                ROS_ERROR("Joint %d over voltage err!\n", i+1);
                break;
            case ERR_MASK_UNDER_VOLTAGE:
                ROS_ERROR("Joint %d under voltage err!\n", i+1);
                break;
            case ERR_MASK_OVER_TEMP:
                ROS_ERROR("Joint %d over temperature err!\n", i+1);
                break;
            case ERR_MASK_HALL:
                ROS_ERROR("Joint %d Hall err!\n", i+1);
                break;
            case ERR_MASK_ENC:
                ROS_ERROR("Joint %d encoder err!\n", i+1);
                break;
            case ERR_MASK_POS_TRACK:
                ROS_ERROR("Joint %d position track err!\n", i+1);
                break;
            case ERR_MASK_CUR_ON:
                ROS_ERROR("Joint %d cunrrent sensor err!\n", i+1);
                break;
            case ERR_MASK_TEMP:
                ROS_ERROR("Joint %d temperature sensor err!\n", i+1);
                break;
            case ERR_MASK_TAG_POS:
                ROS_ERROR("Joint %d target postion over limit err!\n", i+1);
                break;
            case JOINT_CAN_LOSE_ERR:
                ROS_ERROR("Joint %d off-line!\n", i+1);
                break;
            default:
                ROS_ERROR("Joint %d Multi Errors!\n", i+1);
                break;
        }
    }

    return 0;
}

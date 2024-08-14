# 获取机械臂状态`GET_ARM_STATE_DEMO`

## 1.项目介绍

本项目是一个基于机械臂机械臂本体和ROS功能包实现获取机械臂当前控制器版本、关节状态、位姿状态、六维力信息功能，程序将依次执行获取控制器版本信息，获取机械臂状态，获取六维力数据信息的指令，程序执行时，对应的数据信息会打印在终端中，该示例目的是使ROS开发者迅速掌握并灵活运用机械臂。

## 2. 代码结构

```
├── CMakeLists.txt                    <-CMake编译文件
├── launch                            <-启动文件夹
│   └── get_arm_state_demo.launch     <-启动文件
├── LICENSE                           <-版本说明
├── package.xml                       <-依赖描述文件夹
├── README.md                         <-说明文档
└── src                               <-C++源码文件夹
    └── api_Get_Arm_State_demo.cpp    <-源码文件
```

## 3. 项目下载

通过项目链接下载本项目工程文件到本地：通过项目链接下载本项目工程文件到本地：[rm_robot](https://github.com/RealManRobot/rm_robot.git)

## 4. 环境配置

| 项目 | 内容 |
| :-- | :-- |
| 系统 | Ubuntu18.04或20.04 |
| ROS版本 | melodic或noetic |
| 依赖 | ROS1功能包 |

**配置过程**

1. 首先需要准备好Ubuntu18.04或Ubuntu20.04操作系统的虚拟机或其他设备。
2. 安装ROS环境[melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)或[noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)。
3. ROS1功能包安装
    新建工作空间和src文件，将ROS1文件放入src中
    ```
    mkdir -p ~/workspace_ws/src
    ```

    进入工作空间
    ```
    cd ~/workspace_ws
    ```

    编译rm_msgs功能包
    ```
    catkin build rm_msgs
    ```

    声明环境变量
    ```
    source ~/workspace_ws/devel/setup.bash
    ```

    编译所有功能包
    ```
    catkin build
    ```

    再次声明环境变量
    ```
    source ~/workspace_ws/devel/setup.bash
    ```

4. demo功能包安装
  新建工作空间和src文件，将ROS1文件放入src中
    ```
    mkdir -p ~/demo_ws/src
    ```

    进入工作空间
    ```
    cd ~/demo_ws/src
    ```

    编译get_arm_state功能包
    ```
    catkin build get_arm_state
    ```

    声明环境变量
    ```
    source ~/demo_ws/devel/setup.bash
    ```

## 5. 使用指南

* **命令行使用**：
    我们需要在一个终端中启动机械臂的rm_driver功能包。
    ```
    roslaunch rm_driver rm_<arm_type>_driver.launch
    ```
    <arm_type>可以为65、63、eco65、75、gen72，可对照自己使用的设备进行实际选择
    我们需要在另一个终端中启动机械臂的get_arm_state功能包。
    ```
    roslaunch get_arm_state get_arm_state_demo.launch
    ```
* **返回信息**：

    在程序成功运行时将会出现以下提示信息。
    ```
    [ INFO] [1722825911.690500489]: *******Get Arm Software Version Pub show on the rm_driver node.*******       //获取控制器版本运行提示
    [ INFO] [1722825911.699439368]: Product_version is GEN72-BI                                                  //机械臂型号
    [ INFO] [1722825911.699802395]: Plan_version is 7b0156                                                       //控制器版本，7代表自由度，b代表没有六维力，156为控制器程序版本为1.5.6
    [ INFO] [1722825912.693324232]: *******Get Arm State Pub*******                                              //获取机械臂状态运行提示
    [ INFO] [1722825912.704657425]: joint angle state is: [14.971000, -12.197000, 18.629000, -83.942001, -5.688000, -6.010000, 24.881001]  //机械臂当前角度信息

    [ INFO] [1722825912.704842087]: pose Euler angle is: [0.242000, 0.174000, 0.536000, 3.078000, -0.254000, 0.139000]                     //机械臂当前位姿信息（欧拉角）

    [ INFO] [1722825912.705156723]: joint angle state is: [0.261244, -0.212838, 0.325076, -1.464788, -0.099256, -0.104875, 0.434173]       //机械臂当前弧度信息

    [ INFO] [1722825912.705307193]: pose Quaternion is: [0.242178, 0.174678, 0.536134, 0.989331, 0.064833, 0.128479, 0.022668]             //机械臂当前位姿信息(四元数)

    [ INFO] [1722825913.693883637]: *******Get Arm Six Force Pub*******                                          //获取机械臂六维力信息运行提示（存在六维力传感器将在下方进行打印，没有六维力将不会打印数据）
    [ INFO] [1722825914.694311968]: *******All command is over please click ctrl+c end*******                    //所有指令完成后提示
    ```

## 6.关键代码说明

下面是 `api_Get_Arm_State_demo.cpp` 文件的主要功能：

- **初始化**
相关发布订阅信息初始化
    
    
    获取机械臂当前版本指令
    ```
    ros::Publisher test_Get_Arm_Software_Version_pub = nh.advertise<std_msgs::Empty>("/rm_driver/Get_Arm_Software_Version", 10);
    ```

    获取机械臂当前状态指令
    ```
    ros::Publisher test_Get_Arm_Base_State_pub = nh.advertise<rm_msgs::GetArmState_Command>("/rm_driver/GetArmState_Cmd", 10);
    ```

    获取机械臂六维力指令
    ```
    ros::Publisher test_Get_Arm_Six_Force_pub = nh.advertise<std_msgs::Empty>("/rm_driver/GetSixForce_Cmd", 10);
    ```

    订阅机械臂版本信息
    ```
    ros::Subscriber Arm_Software_Version_sub = nh.subscribe("/rm_driver/Get_Arm_Software_Version_Result", 10, Get_Arm_Software_Version_Callback);
    ```

    订阅机械臂当前状态指令(角度+欧拉角)
    ```
    ros::Subscriber Arm_Base_State_sub = nh.subscribe("/rm_driver/ArmCurrentState", 10, Get_Arm_State_Callback);
    ```

    订阅机械臂当前状态指令(弧度+四元数)
    ```
    ros::Subscriber Arm_New_State_sub = nh.subscribe("/rm_driver/Arm_Current_State", 10, GetArmState_Callback);
    ```

    订阅机械臂六维力原始数据
    ```
    ros::Subscriber GetSixForceState_sub = nh.subscribe("/rm_driver/GetSixForce", 10, Get_Six_Force_Callback);
    ```

    订阅机械臂传感器坐标系下的六维力数据
    ```
    ros::Subscriber SixZeroForceState_sub = nh.subscribe("/rm_driver/SixZeroForce", 10, Get_Six_Zero_Force_Callback);
    ```

    订阅机械臂工作坐标系下的传感器数据
    ```
    ros::Subscriber WorkZeroForceState_sub = nh.subscribe("/rm_driver/WorkZeroForce", 10, Get_Work_Zero_Force_Callback);
    ```

    订阅机械臂工具坐标系下的传感器数据
    ```
    ros::Subscriber ToolZeroForceState_sub = nh.subscribe("/rm_driver/ToolZeroForce", 10, Get_Tool_Zero_Force_Callback);
    ```

- **回调函数**
    接收机械臂版本信息，进入消息回调函数
    ```
    void Get_Arm_Software_Version_Callback(const rm_msgs::Arm_Software_Version msg)
    ```

    接收机械臂状态信息，进入消息回调函数（角度+欧拉角）
    ```
    void GetArmState_Callback(const rm_msgs::Arm_Current_State msg)
    ```

    接收机械臂状态信息，进入消息回调函数（弧度+四元数）
    ```
    void Get_Arm_State_Callback(const rm_msgs::ArmState msg)
    ```

    接收机械臂六维力状态信息，进入消息回调函数（原始数据）
    ```
    void Get_Six_Force_Callback(const rm_msgs::Six_Force msg)
    ```

    接收机械臂六维力状态信息，进入消息回调函数（系统受力数据）
    ```
    void Get_Six_Zero_Force_Callback(const rm_msgs::Six_Force msg)
    ```

    接收机械臂六维力状态信息，进入消息回调函数（工作坐标系受力数据）
    ```
    void Get_Work_Zero_Force_Callback(const rm_msgs::Six_Force msg)
    ```

    接收机械臂六维力状态信息，进入消息回调函数（工具坐标系受力数据）
    ```
    void Get_Tool_Zero_Force_Callback(const rm_msgs::Six_Force msg)
    ```

- **发布获取控制器版本指令**
发布获取控制器版本发布指令。

    ```
    test_Get_Arm_Software_Version_pub.publish(empty_value);
    ```

- **发布获取机械臂状态指令**
发布获取机械臂状态指令。

    ```
    test_Get_Arm_Base_State_pub.publish(command);
    ```

- **发布获取六维力指令**
发布获取六维力指令。

    ```
    test_Get_Arm_Six_Force_pub.publish(empty_value);
    ```

## 7. 许可证信息

* 具体许可证内容请参见`LICENSE`文件。

# 力位混合控制规划`ARM_FORCE_POSITION_CONTROL_DEMO`

## 1.项目介绍

本项目是一个基于RM65和RM75机械臂和ROS功能包实现力位混合控制规划运动功能（该功能适用于类似于MoveL的笛卡尔运动，不适用与MoveJ等关节运动），程序在执行时会依次执行开启力位混合控制，进行笛卡尔运动，关闭力位混合控制的操作，目的是使ROS开发者迅速掌握并灵活运用机械臂。

## **2. 代码结构**

```
├── CMakeLists.txt                           <-CMake编译文件
├── launch                                   <-启动文件夹
│   └── force_position_control_demo.launch   <-启动文件
├── LICENSE                                  <-版本说明
├── package.xml                              <-依赖描述文件夹
├── README.md                                <-说明文档
└── src                                      <-C++源码文件夹
    └── api_force_position_control_demo.cpp  <-源码文件
```

## 3. 项目下载

通过项目链接下载本项目工程文件到本地：[rm_robot](https://github.com/RealManRobot/rm_robot.git)

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

## 5. 使用指南

* **命令行使用**：
    我们需要在一个终端中启动机械臂的rm_driver功能包。
    ```
    roslaunch rm_driver rm_<arm_type>_driver.launch
    ```
    <arm_type>可以为65、63、eco65、75、gen72，可对照自己使用的设备进行实际选择
    我们需要在另一个终端中启动机械臂的force_position_control功能包。
    ```
    roslaunch force_position_control force_position_control_demo.launch
    ```
    
    >若非RM65、RM75机械臂可能会出现无法到达点位的情况，为正常现象。

* **返回信息**：

    在程序成功运行时将会出现以下提示信息。
    ```
    [ INFO] [1722825453.472184452]: MoveJ_P start.                                  //MoveJ_P运动开始时的提示信息
    [ INFO] [1722825454.296426967]: The first trajectory MoveJ_P has been executed  //执行完成提示
    [ INFO] [1722825454.296717308]: Set Force Position start.                       //设置力位混合运动开始时的提示信息
    [ INFO] [1722825454.303498227]: ForcePosition set success                       //执行完成提示
    [ INFO] [1722825455.297280029]: MoveL start                                     //MoveL运动开始时的提示信息
    [ INFO] [1722825456.116331900]: MoveL has been executed                         //执行完成提示
    [ INFO] [1722825456.116422581]: Stop Force Position start.                      //停止力位混合运动时的提示信息
    [ INFO] [1722825456.121557936]: StopForcePosition set success                   //执行完成提示
    [ INFO] [1722825456.121631421]: *******All command is over please click ctrl+c end******* //所有指令完成后提示
    ```

## 6.关键代码说明：

下面是 `api_force_position_control_demo.cpp` 文件的主要功能：

- **初始化**
相关发布订阅信息初始化

    空间规划指令Publisher
    ```
    ros::Publisher MoveJ_P_pub = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);
    ```

    设置六维力指令Publisher
    ```
    ros::Publisher SetForcePosition_pub = nh.advertise<rm_msgs::Set_Force_Position>("/rm_driver/SetForcePosition_Cmd", 10);
    ```

    设置停止六维力指令Publisher
    ```
    ros::Publisher StopForcePosition_pub = nh.advertise<std_msgs::Empty>("/rm_driver/StopForcePosition_Cmd", 10);
    ```

    直线规划指令Publisher
    ```
    moveL_pub = nh.advertise<rm_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);
    ```

    订阅机械臂执行状态话题
    ```
    ros::Subscriber PlanState_sub = nh.subscribe("/rm_driver/Plan_State", 10, PlanState_Callback);
    ```

    订阅机械臂力位混合设置结果
    ```
    ros::Subscriber SetForcePosition_sub = nh.subscribe("/rm_driver/SetForcePosition_Result", 10, SetForcePosition_Callback);
    ```

    订阅机械臂力位混合停止结果
    ```
    ros::Subscriber StopForcePosition_sub = nh.subscribe("/rm_driver/StopForcePosition_Result", 10, StopForcePosition_Callback);
    ```

- **回调函数**
    
    接收到订阅的机械臂执行状态消息后，会进入消息回调函数
    ```
    void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
    ```

    接收到设置机械臂力位混合执行状态消息后，会进入消息回调函数
    ```
    void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
    ```
    
    接收到停止机械臂力位混合执行状态消息后，会进入消息回调函数
    ```
    void SetForcePosition_Callback(const std_msgs::Bool& msg)
    ```

- **发布MoveJ_P指令**
发布MoveJ_P指令使机械臂运动到目标位姿。

    ```
    MoveJ_P_pub.publish(moveJ_P_TargetPose);
    ```

- **发布开启力位混合指令**
设置机械臂开启力位混合指令。

    ```
    SetForcePosition_pub.publish(setForce_value);
    ```

- **发布MoveL指令**
发布MoveL指令使机械臂运动到目标位姿。

    ```
    moveL_pub.publish(moveL_TargetPose);
    ```

- **发布关闭力位混合指令**
发布MoveL指令使机械臂运动到目标位姿。

    ```
    StopForcePosition_pub.publish(empty_value);
    ```

## 7.许可证信息

* 具体许可证内容请参见`LICENSE`文件。

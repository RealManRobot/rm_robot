# 机械臂运动控制`API_CONTROL_ARM_MOVE_DEMO`

## 1.项目介绍

本项目是一个基于RM65、RM75机械臂和ROS功能包实现MoveJ、MoveJ_P、MoveL、MoveC规划运动功能，在程序执行时将依次执行关节运动MoveJ指令，位姿运动MoveJ_P指令、直线运动MoveL指令，圆弧运动MoveC指令，在运动开始和结束时终端都会收到相关提示，目的是使ROS开发者迅速掌握并灵活运用机械臂。

## 2. 代码结构

```
├── CMakeLists.txt                    <-CMake编译文件
├── launch                            <-启动文件夹
│   └── rm_65_move_demo.launch        <-启动文件(RM65)
    └── rm_75_move_demo.launch        <-启动文件(RM75)
├── LICENSE                           <-版本说明
├── package.xml                       <-依赖描述文件夹
├── README.md                         <-说明文档
└── src                               <-C++源码文件夹
    └── api_move_demo.cpp             <-源码文件
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

## 5. 快速使用

* **命令行使用**：
    我们需要在一个终端中启动机械臂的rm_driver功能包。
    ```
    roslaunch rm_driver rm_<arm_type>_driver.launch
    ```
    由于不同构型的机械臂其可到达的点位和位姿各不相同，这里分别对65、75进行了点位的适配，<arm_type>可以为65、75，可对照自己使用的设备进行实际选择。
    之后我们需要在另一个终端中启动机械臂的control_arm_move功能包。
    若为RM65系列机械臂执行如下指令:

    ```
    roslaunch control_arm_move rm_65_move_demo.launch
    ```

    若为RM75系列机械臂执行如下指令:

    ```
    roslaunch control_arm_move rm_75_move_demo.launch
    ```

    >若非RM65、RM75机械臂可能会出现无法到达点位的情况，为正常现象。

* **返回信息**：

    在程序成功运行时将会出现以下提示信息。
    ```
    [ INFO] [1722565910.866307974]: *******arm_dof = 7           //机械臂当前设置的自由度信息将会在这里进行打印
    [ INFO] [1722565912.871168796]: *******Plan MoveJ Start      //MoveJ运动开始时的提示信息
    [ INFO] [1722565915.832463777]: *******Plan MoveJ State OK   //MoveJ运动结束时的提示信息
    [ INFO] [1722565916.832557775]: *******Plan MoveJ_P Start    //MoveJ_P运动开始时的提示信息
    [ INFO] [1722565920.192072186]: *******Plan MoveJ_P State OK //MoveJ_p运动结束时的提示信息
    [ INFO] [1722565921.192211738]: *******Plan MoveL Start      //MoveL运动开始时的提示信息
    [ INFO] [1722565922.018137911]: *******Plan MoveL State OK   //MoveL运动结束时的提示信息
    [ INFO] [1722565922.192413396]: *******Plan MoveC Start      //MoveC运动开始时的提示信息
    [ INFO] [1722565945.932390913]: *******Plan MoveC State OK   //MoveC运动结束时的提示信息
    ```

## 6.关键代码说明

下面是 `api_move_demo.cpp` 文件的主要功能：

- **初始化**
相关发布订阅信息初始化
    
    为moveJ空间规划指令创建Publisher
    ```
    ros::Publisher moveJ_pub = nh.advertise<rm_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);
    ```

    为moveJ_P空间规划指令创建Publisher
    ```
    ros::Publisher moveJ_P_pub = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);
    ```

    为moveL指令创建Publisher
    ```
    ros::Publisher moveL_pub = nh.advertise<rm_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);
    ```

    为moveC指令创建Publisher
    ```
    ros::Publisher moveC_pub = nh.advertise<rm_msgs::MoveC>("/rm_driver/MoveC_Cmd", 10);
    ```

    订阅机械臂执行状态话题
    ```
    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);
    ```

- **回调函数**
接收到订阅的机械臂执行状态消息后，会进入消息回调函数

    ```ROS
    void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
    ```

- **发布MoveJ指令**
发布MoveJ指令使机械臂运动到目标位姿。

    ```ROS
    moveJ_pub.publish(moveJ_BeginPose);
    ```

- **发布MoveJ_P指令**
发布MoveJ_P指令使机械臂运动到目标位姿。

    ```ROS
    moveJ_P_pub.publish(moveJ_P_TargetPose);
    ```

- **发布MoveL指令**
发布MoveL指令使机械臂运动到目标位姿。

    ```ROS
    moveL_pub.publish(moveL_TargetPose);
    ```

- **发布MoveC指令**
发布MoveC指令使机械臂运动到目标位姿。

    ```ROS
    moveC_pub.publish(moveC_TargetPose);
    ```

## 7. 许可证信息

* 具体许可证内容请参见`LICENSE`文件。
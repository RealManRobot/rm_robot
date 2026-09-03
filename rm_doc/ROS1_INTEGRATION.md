# ROS 1 V2.6.0 派生更新整合说明

这是基于 V2.6.0 的本地整合更新，位于 `V2.7.0` 分支，尚未发布，也未创建新的版本标签。
目标仓库是 `ROS/rm_robotT1/rm_robot`；另一份 `ROS/rm_robot` 仅用作 DAE 资源来源。

## 更新来源与接口选择

| 来源 | 整合内容 |
|---|---|
| `31aaca2` | TCP/UDP 接收边界修复、UDP Plus 类型解析、整数范围校验及 scalar/array `err` 兼容 |
| 原工作区改动 | `Hand_Pos`、`Hand_Angle.num_dof`、显式 1/6 DOF 跟随和消息构建依赖 |
| `45bc2a4` | 已核对；按当前接口选择不整合四种运动指令的 `r` 入参 |
| `bfa620f` | STL、URDF/Xacro 物理定义和六套 Gazebo MoveIt 控制器入口 |
| `9e1b560` | 34 个 DAE、纹理及视觉配置和启动入口修正 |

跟随指令统一使用消息中的显式 `num_dof`，不使用自动 DOF 缓存，也不增加启动时的基础信息查询。
UDP 基础信息只用于反馈，不会改变控制命令的数组长度。

## 已处理的排查问题

1. 正常 UDP Plus 对象不再通过 `valuestring` 调用 `strcmp`；`offline`/`disable` 字符串不会进入对象解析。
2. UDP 预留终止符并检查短包；TCP 写入前检查容量，超长帧丢弃到下一换行后恢复，断开连接时清除半帧。
3. 状态必填字段严格校验；能力字段可以缺少，但存在且类型或范围错误时拒绝发布。版本字段读取字符串，兼容整数版本号。
4. 手部位置跟随、Plus mode/touch 的设置和查询回执分别结束分支，不再串发其他结果。
5. 保留显式 DOF 消息，补齐注册、生成依赖、调用端示例和重建说明；打包必须包括新增的 `Hand_Pos.msg`。
6. 删除自动 DOF 控制缓存，消除校验失败仍修改控制长度的问题。
7. 检查所有可选数组的解析结果，拒绝小数、越界整数、非数字和错误字段类型；不发布半解析消息。
8. 采用 `bfa620f` 的成套物理定义；检查 14 对静态 URDF/Xacro 的关节、惯量和碰撞定义一致性。

## 消息迁移与重建

`Hand_Angle` 的 ROS 1 MD5/布局已改变。
位置跟随更换为新增的 `Hand_Pos`。发布端、驱动及其他订阅端必须使用同一套重新生成的消息。
仅重启驱动或者仅重新 source 旧工作区无法完成迁移。

请将完整源码放入新的 catkin 工作区 `src/rm_robot`，包括新增消息、模型和纹理。
在该工作区根目录执行（本机没有执行这些 ROS 1 命令）：

```bash
source /opt/ros/noetic/setup.bash
catkin build rm_msgs
source devel/setup.bash
catkin build
source devel/setup.bash
```

Melodic 环境使用对应的 `/opt/ros/melodic/setup.bash`。所有外部客户端也需使用这些消息重新构建。
不要将另一份仓库或旧工作区的 `devel/setup.bash` 叠加到本工作区后面。

| 话题 | 消息 | DOF 规则 |
|---|---|---|
| `/rm_driver/Hand_FollowPos` | `rm_msgs/Hand_Pos` | `num_dof` 必填 1 或 6，发送前 N 项 |
| `/rm_driver/Hand_FollowAngle` | `rm_msgs/Hand_Angle` | `num_dof` 必填 1 或 6，发送前 N 项 |
| `/rm_driver/Hand_SetAngle` | `rm_msgs/Hand_Angle` | 仍为六自由度设置；接受 6，兼容旧调用默认值 0，拒绝 1 |

`num_dof` 必须匹配末端实际自由度。六自由度灵巧手只操作一个手指时也应填写 6，按设备协议填写其余项。
跟随接口不使用 `block`；它是兼容保留字段，不能据此等待物理到位。

下面仅展示消息格式；执行会向真实末端发送指令，应先确认连接的型号、数值含义和动作条件：

```bash
# 仅适用于实际一自由度末端的位置跟随
rostopic pub -1 /rm_driver/Hand_FollowPos rm_msgs/Hand_Pos \
  '{hand_angle: [1000, 0, 0, 0, 0, 0], num_dof: 1, block: false}'

# 六自由度角度跟随格式
rostopic pub -1 /rm_driver/Hand_FollowAngle rm_msgs/Hand_Angle \
  '{hand_angle: [0, 0, 0, 0, 0, 0], num_dof: 6, block: false}'
```

`MoveJ`、`MoveL`、`MoveC`、`MoveJ_P` 保持 V2.6.0 的消息与调用接口，不增加 `r` 入参；
底层 JSON 继续固定发送 `r=0`。代码仅添加一句扩展提示：如需设置交融半径，请在运动消息、回调及组包函数中添加 `r` 参数。
原有 `Movel_Offset` 的 `r` 接口及输入范围校验保留。

## UDP Plus 反馈

`Udp_plus_state`/`Udp_plus_base` 配置只决定实时上报内容，不决定 TCP 跟随指令能否下发。
两项默认仍为 false。在线对象按类型解析；没有启用时字段可缺少，离线或禁用字符串不会产生伪造的在线状态。
错误数组只阻止该末端消息发布，其他有效的机械臂反馈仍可处理。

`set_state:true` 是控制器命令回执，不能代替末端位置反馈或物理到位判断。

## 模型与 Gazebo

物理定义和 STL 来自 `bfa620f`，保留其关节、惯量、限位及碰撞更新，没有用 DAE 提交覆盖这些数据。
这次整合没有重新标定物理参数；与具体硬件的符合性仍需确认。

DAE 视觉需要适配更新后的连杆坐标：RM75 的 Link1–6、ECO65 的 Link2 和 ECO63 的 Link2 已调整。
导入器检查发现以下部件的旧 DAE 与更新后的 STL 几何边界存在明显差别，因此继续使用更新后的 STL 视觉：

- RM65 系列底座。
- ECO63 系列底座和 6FB 末端。
- ECO65 系列底座和 Link1。
- 标准 RM75 的 Link7。

其他适配的部件保留 DAE 彩色视觉。碰撞仍使用更新后的 STL；未修改 DAE 文件内部的几何、材质或单位。
这不是将不同物理模型通过缩放强行对齐。

Gazebo 使用 `moveit_planning_execution.launch` 加载 `controllers_gazebo.yaml`。
真机入口继续使用 `controllers.yaml`，无需为切换仿真而手工改写共享控制器配置。

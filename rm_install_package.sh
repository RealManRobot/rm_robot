#!/usr/bin/env bash
# install_ros_deps.sh  —— 一键安装 ROS 常用可视化与仿真依赖（18.04/20.04 通用）
set -e

#--------------------------------------------------
# 0. 要求必须是 root 或具有 sudo 权限
#--------------------------------------------------
if ! sudo -n true 2>/dev/null; then
    echo "❌ 请先配置好 sudo 免输密码或手动输入密码继续。"
fi

#--------------------------------------------------
# 1. 自动检测 ROS 发行版（按系统版本）
#--------------------------------------------------
UBUNTU_VER=$(lsb_release -rs)   # 18.04 / 20.04
case "$UBUNTU_VER" in
    18.04) ROS_DISTRO=melodic ;;
    20.04) ROS_DISTRO=noetic  ;;
        *) echo "❌ 本脚本仅支持 Ubuntu 18.04/20.04"; exit 1 ;;
esac

echo "=> 检测到 Ubuntu $UBUNTU_VER，对应 ROS $ROS_DISTRO"

# 如果系统里还没装 ROS，提示先装
if [ ! -d "/opt/ros/$ROS_DISTRO" ]; then
    echo "❌ 未发现 /opt/ros/$ROS_DISTRO，请先完成 ROS 安装后再运行本脚本。"
    exit 1
fi

# 加载环境，防止用户没 source
source /opt/ros/$ROS_DISTRO/setup.bash

#--------------------------------------------------
# 2. 更新 APT
#--------------------------------------------------
echo "=> 更新 APT 索引"
sudo apt-get update -qq

#--------------------------------------------------
# 3. 按版本给出真实存在的包名
#--------------------------------------------------
case "$ROS_DISTRO" in
melodic)   # Ubuntu 18.04
    PKG_LIST="
        ros-melodic-rviz
        ros-melodic-gazebo-ros-pkgs
        ros-melodic-gazebo-ros-control
        ros-melodic-moveit
        ros-melodic-moveit-visual-tools
        ros-melodic-moveit-setup-assistant
        ros-melodic-moveit-ros-visualization
    "
    ;;
noetic)    # Ubuntu 20.04
    PKG_LIST="
        ros-noetic-rviz
        ros-noetic-gazebo-ros-pkgs
        ros-noetic-gazebo-ros-control
        ros-noetic-moveit
        ros-noetic-moveit-visual-tools
        ros-noetic-moveit-setup-assistant
        ros-noetic-moveit-ros-visualization
    "
    ;;
esac

#--------------------------------------------------
# 4. 统一安装
#--------------------------------------------------
echo "=> 安装 ROS 依赖包（较多，请耐心等待）"
sudo apt-get install -y $PKG_LIST

#--------------------------------------------------
# 5. 把环境变量写入 ~/.bashrc
#--------------------------------------------------
grep -F "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc >/dev/null 2>&1 || {
    echo "=> 将 ROS 环境写入 ~/.bashrc"
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
}

echo
echo "✅ 依赖安装完成！请重新打开终端或执行  source ~/.bashrc  使环境生效。"

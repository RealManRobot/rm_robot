#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit__demo', anonymous=True)
arm = MoveGroupCommander('arm')
rospy.sleep(0.5)
arm.set_named_target('forward')

plan=arm.plan()
arm.execute(plan,wait=False)
rospy.loginfo("move home finished")
rospy.sleep(0.5)
arm.stop()
rospy.loginfo("stoped")
rospy.sleep(0.5)
moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)

# ===============c++============
# #include <moveit/move_group_interface/move_group_interface.h>
# int main(int argc, char **argv)
# {
#   ros::init(argc, argv, "moveit_custom_demo");
#   ros::NodeHandle node_handle; 

#   moveit::planning_interface::MoveGroupInterface group("arm");
#   group.setNamedTarget("home");
#   group.move();
#   geometry_msgs::Pose target_pose1;
#   target_pose1.orientation.w =-1.0;
#   target_pose1.orientation.x= 0;
#   target_pose1.orientation.y =0;
#   target_pose1.orientation.z =0;

#   target_pose1.position.x = 0.2;
#   target_pose1.position.y = 0;
#   target_pose1.position.z = 0.28;
#   group.setPoseTarget(target_pose1);


# moveit::planning_interface::MoveGroupInterface::Plan my_plan;
# moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan)
# group.asyncexecute(my_plan);
# sleep(1);
# group.stop();
# sleep(1);

#   ros::shutdown(); 
#   return 0;
# }


    

    

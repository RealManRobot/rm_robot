/**
 * 在环境中添加或移除对象
 * 为机器人附加或拆卸物体
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_ros_api_demo");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;

  //通过MoveItVisualTools插件控制程序运行
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  //广播所需要的话题
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // 定义附加对象消息
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "Link6";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "Link6";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  /* 定义物体形状及尺寸大小 */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  //附加对象需要有一个添加操作
  attached_object.object.operation = attached_object.object.ADD;

  //因为要将物体连接到机器人的手上以模拟拿起物体，所以设置碰撞检查器忽略物体和机器人手之间的碰撞
  attached_object.touch_links = std::vector<std::string>{ "Link4", "Link5", "Link6" };

  // 添加物体到场景中
  ROS_INFO("Adding the object into the world at the location of the hand.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  /**
   * 同步更新与异步更新
   * 1.一种方式是通过rosservice调用发送一个diff,直到diff被应用(同步机制)
   * 2.另一种方式是通过话题发送diff,无论是否应用都继续进行
   */
  ros::ServiceClient planning_scene_diff_client =
      node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);

  /**
   * 将物体附加到机器人上
   * 当机器人从环境中拾取物体时,需要将物体"附着"到机器人上,以便处理机器人模型的任何部件知道对附着的物体进行说明,比如进行碰撞检查.
   * 1.首先删除原对象
   * 2.然后将对象附着到机器人上
   */

  /* First, define the REMOVE object message*/
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "Link1";
  remove_object.operation = remove_object.REMOVE;

  ROS_INFO("Attaching the object to the hand and removing it from the world.");
  // 确定diff消息没有包含其他对象
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  /**
   * 从机器人接触附着物体,需要两步:
   * 1.解除物体
   * 2.重新将物体引入到环境中
   */

  /* First, define the DETACH object message*/
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "Link2";
  detach_object.object.operation = attached_object.object.REMOVE;


  ROS_INFO("Detaching the object from the robot and returning it to the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 将物体从碰撞环境中移除
  ROS_INFO("Removing the object from the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher.publish(planning_scene);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");

  ros::shutdown();
  return 0;
}

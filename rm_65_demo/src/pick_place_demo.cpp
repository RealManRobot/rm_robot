//
// Created by ubuntu on 21-8-11.
//
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // 创建环境中的物体(两个桌子和一个抓取的目标物体)
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";

    // 设置物体table1的形状及相关尺寸
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    // 设置物体table1的位置
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.45;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;

    collision_objects[0].operation = collision_objects[0].ADD;

    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";

    // 设置物体table2的形状及相关尺寸
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    // 设置物体table2的位置
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.45;
    collision_objects[1].primitive_poses[0].position.z = 0.2;

    collision_objects[1].operation = collision_objects[1].ADD;


    // 创建并配置一个抓取的目标物体
    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].id = "object";

    // 设置目标物体的形状和相关尺寸
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;

    // 设置目标物体的位置
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.44;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rm65_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();

//    ros::WallDuration(1.0).sleep();

    //创建运动规划的情景，等待创建完成
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // 初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface group("arm");
    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.setGoalPositionTolerance(0.02);
    group.setGoalOrientationTolerance(0.05);
    sleep(5.0);

    // 在场景中创建相关物体
    addCollisionObjects(planning_scene_interface);

    // 等待一段时间来初始化ROS
    ros::WallDuration(1.0).sleep();

    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose1;
//    target_pose1.orientation.w = 0.726282;
//    target_pose1.orientation.x= 4.04423e-07;
//    target_pose1.orientation.y = -0.687396;
//    target_pose1.orientation.z = 4.81813e-07;
//    target_pose1.orientation.w = 0.7;
//
//    target_pose1.position.x = 0.38;
//    target_pose1.position.y = 0;
//    target_pose1.position.z = 0.53;

    target_pose1.orientation.w = 0.703184;
    target_pose1.orientation.x= 1.42057e-05;
    target_pose1.orientation.y = 0.711007;
    target_pose1.orientation.z = -1.912e-05;

    target_pose1.position.x = 0.406138;
    target_pose1.position.y = 0.00100714;
    target_pose1.position.z = 0.478058;
    group.setPoseTarget(target_pose1, "Link6");

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"target_pose1":"FAILED");

    //让机械臂按照规划的轨迹开始运动。
    if(success)
    {
        group.execute(my_plan);

        ros::WallDuration(1.0).sleep();

        //创建一个附加到Link6上的物体
        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = "Link6";
        /* The header must contain a valid TF frame*/
        attached_object.object.header.frame_id = "base_link";
        /* 设置物体ID */
        attached_object.object.id = "object1";

        /* 设置目标物体的位置 */
        geometry_msgs::Pose pose;
//    pose.orientation.w = 1.0;
        pose.position.x = 0.44;
        pose.position.y = 0;
        pose.position.z = 0.5;

        /* 定义物体的形状和相关尺寸 */
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.02;
        primitive.dimensions[1] = 0.02;
        primitive.dimensions[2] = 0.2;

        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(pose);

        // 将一个对象附加到机器人要指定为ADD操作的对应操作
        attached_object.object.operation = attached_object.object.ADD;

        // 因为要将物体连接到机器人的手上以模拟拿起物体所以配置碰撞检查器忽略物体和机器人手之间的碰撞
        attached_object.touch_links = std::vector<std::string>{"Link6"};

        // 将创建的附加物体应用到场景中
        planning_scene_interface.applyAttachedCollisionObject(attached_object);
        // 从场景中移除原来的目标物体
        planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"object"});

//    ros::WallDuration(1.0).sleep();

        // 设置放置点的目标位置
        geometry_msgs::Pose target_pose2;
        target_pose2.orientation.w = 0.501046;
        target_pose2.orientation.x= -0.49893;
        target_pose2.orientation.y = 0.49938;
        target_pose2.orientation.z = 0.500641;

        target_pose2.position.x = -0.000392824;
        target_pose2.position.y = 0.394717;
        target_pose2.position.z = 0.504879;

//    target_pose2.orientation.w = 0.500196;
//    target_pose2.orientation.x= -0.49973;
//    target_pose2.orientation.y = 0.500227;
//    target_pose2.orientation.z = 0.499847;
//
//    target_pose2.position.x = -0.000394317;
//    target_pose2.position.y = 0.401346;
//    target_pose2.position.z = 0.467557;

//    target_pose2.orientation.w = 0.498372;
//    target_pose2.orientation.x= -0.501688;
//    target_pose2.orientation.y = 0.50205;
//    target_pose2.orientation.z = 0.497876;
//
//    target_pose2.position.x = -0.00039465;
//    target_pose2.position.y = 0.394717;
//    target_pose2.position.z = 0.504879;
        group.setPoseTarget(target_pose2, "Link6");

        // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
        moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
        moveit::planning_interface::MoveItErrorCode success2 = group.plan(my_plan2);

        ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"target_pose2":"FAILED");

        //让机械臂按照规划的轨迹开始运动。
        if(success2)
        {
            group.execute(my_plan2);

            ros::WallDuration(1.0).sleep();

            //创建一个在放置点处的物体
            std::vector<moveit_msgs::CollisionObject> collision_objects1;
            collision_objects1.resize(1);

            collision_objects1[0].id = "object2";
            collision_objects1[0].header.frame_id = "base_link";

            // 设置物体的形状及相关尺寸
            collision_objects1[0].primitives.resize(1);
            collision_objects1[0].primitives[0].type = collision_objects1[0].primitives[0].BOX;
            collision_objects1[0].primitives[0].dimensions.resize(3);
            collision_objects1[0].primitives[0].dimensions[0] = 0.02;
            collision_objects1[0].primitives[0].dimensions[1] = 0.02;
            collision_objects1[0].primitives[0].dimensions[2] = 0.2;

            // 设置物体位置
            collision_objects1[0].primitive_poses.resize(1);
            collision_objects1[0].primitive_poses[0].position.x = 0;
            collision_objects1[0].primitive_poses[0].position.y = 0.44;
            collision_objects1[0].primitive_poses[0].position.z = 0.5;

            collision_objects1[0].operation = collision_objects1[0].ADD;


            //创建一个用于解除附加到机械臂Link6的附加物体
            moveit_msgs::AttachedCollisionObject detach_object;
            detach_object.link_name = "Link6";
            /* The header must contain a valid TF frame*/
            detach_object.object.header.frame_id = "base_link";
            /* The id of the object */
            detach_object.object.id = "object1";
            detach_object.object.operation = attached_object.object.REMOVE;

            //达到放置点后移除附加物体然后在放置点加入放置的物体
            planning_scene_interface.applyAttachedCollisionObject(detach_object);
            planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"object1"});
            planning_scene_interface.addCollisionObjects(collision_objects1);

            //设置机械臂返回zero姿态
            group.setNamedTarget("zero");
            // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
            moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
            moveit::planning_interface::MoveItErrorCode success3 = group.plan(my_plan3);

            ROS_INFO("Visualizing plan 3 (pose goal) %s",success3?"zero":"FAILED");

            //让机械臂按照规划的轨迹开始运动。
            if(success3)
                group.execute(my_plan3);
        }
    }

    ros::waitForShutdown();

    return 0;
}

// Adaptado para UR5e com Robotiq 2F-85
// Define:
// - planning group: "manipulator"
// - joints da garra: "robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint"
// - frame base: "base_link", EEF: "tool0"

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const std::string ARM_GROUP   = "manipulator";
const std::string GRIPPER_J1  = "robotiq_85_left_knuckle_joint";
const std::string GRIPPER_J2  = "robotiq_85_right_knuckle_joint";
const std::string BASE_FRAME  = "world";
const std::string EEF_FRAME   = "tool0";

void openGripper(trajectory_msgs::JointTrajectory& posture) {
  posture.joint_names = {GRIPPER_J1, GRIPPER_J2};
  posture.points.resize(1);
  posture.points[0].positions = {0.04, 0.04};
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture) {
  posture.joint_names = {GRIPPER_J1, GRIPPER_J2};
  posture.points.resize(1);
  posture.points[0].positions = {0.00, 0.00};
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group) {
  std::vector<moveit_msgs::Grasp> grasps(1);

  grasps[0].grasp_pose.header.frame_id = BASE_FRAME;
  tf2::Quaternion q;
  q.setRPY(-M_PI/2, 0, -M_PI/2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(q);
  grasps[0].grasp_pose.pose.position.x = 0.45;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 1.06;

  grasps[0].pre_grasp_approach.direction.header.frame_id = BASE_FRAME;
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.05;
  grasps[0].pre_grasp_approach.desired_distance = 0.1;

  grasps[0].post_grasp_retreat.direction.header.frame_id = BASE_FRAME;
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.05;
  grasps[0].post_grasp_retreat.desired_distance = 0.2;

  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table");
  move_group.pick("pick_cube", grasps);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& psi) {
  std::vector<moveit_msgs::CollisionObject> objs(2);

  // Mesa
  objs[0].id = "table";
  objs[0].header.frame_id = BASE_FRAME;
  objs[0].primitives.resize(1);
  objs[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  objs[0].primitives[0].dimensions = {1.5, 0.8, 0.03};
  objs[0].primitive_poses.resize(1);
  objs[0].primitive_poses[0].position.x = 0.33;
  objs[0].primitive_poses[0].position.y = 0.0;
  objs[0].primitive_poses[0].position.z = 0.35;
  objs[0].primitive_poses[0].orientation.w = 1.0;
  objs[0].operation = objs[0].ADD;

  // Cubo
  objs[1].id = "pick_cube";
  objs[1].header.frame_id = BASE_FRAME;
  objs[1].primitives.resize(1);
  objs[1].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  objs[1].primitives[0].dimensions = {0.04, 0.04, 0.04};
  objs[1].primitive_poses.resize(1);
  objs[1].primitive_poses[0].position.x = 0.45;
  objs[1].primitive_poses[0].position.y = 0.0;
  objs[1].primitive_poses[0].position.z = 0.35 + 0.04/2;  // cubo em cima da mesa
  objs[1].primitive_poses[0].orientation.w = 1.0;
  objs[1].operation = objs[1].ADD;

  psi.applyCollisionObjects(objs);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ur5e_pick_place_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface psi;
  moveit::planning_interface::MoveGroupInterface group(ARM_GROUP);
  group.setEndEffectorLink(EEF_FRAME);
  group.setPlanningTime(10.0);

  ros::WallDuration(1.0).sleep();
  addCollisionObjects(psi);
  ros::WallDuration(1.0).sleep();
  pick(group);
  ros::waitForShutdown();
  return 0;
}
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import yaml
import moveit_commander
from geometry_msgs.msg import PoseStamped
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from tf.transformations import quaternion_from_euler
from math import pi
from visualization_msgs.msg import Marker

# ------------------------------------------------------------------
# CONFIGURAÇÕES
# ------------------------------------------------------------------
ROBOT   = "robot"                  # nome que aparece em /gazebo/model_states

# dimensões da lata reduzida (m)
BEER_HEIGHT = 0.16
BEER_RADIUS = 0.035

# ------------------------------------------------------------------
# AUXÍLIO GARRA
# ------------------------------------------------------------------
def open_gripper(client):
    goal = GripperCommandGoal()
    goal.command.position = 0.0
    goal.command.max_effort = 100.0
    client.send_goal(goal)
    client.wait_for_result()

def close_gripper(client):
    goal = GripperCommandGoal()
    goal.command.position = 0.15
    goal.command.max_effort = 100.0
    client.send_goal(goal)
    client.wait_for_result()

# ------------------------------------------------------------------
# ATTACH / DETACH (beer)
# ------------------------------------------------------------------
def attach_beer():
    rospy.wait_for_service('/link_attacher_node/attach')
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    for finger_link in ["left_inner_finger", "right_inner_finger"]:
        req = AttachRequest()
        req.model_name_1 = ROBOT
        req.link_name_1  = finger_link
        req.model_name_2 = "beer"
        req.link_name_2  = "link"
        attach_srv(req)
    rospy.loginfo("Beer anexada aos dois dedos.")

def detach_beer():
    rospy.wait_for_service('/link_attacher_node/detach')
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    for finger_link in ["left_inner_finger", "right_inner_finger"]:
        req = AttachRequest()
        req.model_name_1 = ROBOT
        req.link_name_1  = finger_link
        req.model_name_2 = "beer"
        req.link_name_2  = "link"
        detach_srv(req)
    rospy.loginfo("Beer solta dos dois dedos.")

# ------------------------------------------------------------------
# ADICIONA OBJETOS NA PLANNING SCENE
# ------------------------------------------------------------------
def add_scene_objects(yaml_path, scene):
    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)
    for obj in cfg["planning_scene_objects"]:
        pose = PoseStamped()
        p_cfg = obj.get("pose", {})
        pose.header.frame_id = p_cfg.get("frame_id", "world")
        px, py, pz = p_cfg.get("position", [0.0, 0.0, 0.0])
        ox, oy, oz, ow = p_cfg.get("orientation", [0.0, 0.0, 0.0, 1.0])
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = px, py, pz
        pose.pose.orientation.x, pose.pose.orientation.y = ox, oy
        pose.pose.orientation.z, pose.pose.orientation.w = oz, ow
        size = tuple(obj["dimensions"])
        scene.add_box(obj["name"], pose, size=size)

# ------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------
def main():
    rospy.init_node("topdown_pick")
    moveit_commander.roscpp_initialize([])

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm   = moveit_commander.MoveGroupCommander("manipulator")

    # Action client da garra
    gripper_client = actionlib.SimpleActionClient(
        '/robotiq_2f_85_gripper_controller/gripper_cmd',
        GripperCommandAction
    )
    rospy.loginfo("Aguardando servidor da garra…")
    gripper_client.wait_for_server()
    rospy.loginfo("Servidor da garra conectado.")
    rospy.sleep(2.0)

    # Adiciona objetos do YAML (beer com dimensões atualizadas)
    yaml_path = os.path.expanduser(
        "~/ur5e_mtc_ws/src/ur5e_robotiq_85_mtc/ur5e_robotiq_85_mtc_pkg/config/mtc/pick_and_place.yaml"
    )
    add_scene_objects(yaml_path, scene)
    rospy.sleep(1.0)

    # ------------------ PARÂMETROS DA TAREFA ------------------
    x = 0.5
    y = 0.00
    mesa_z  = 0.10
    beer_h  = BEER_HEIGHT
    margin  = 0.08
    center_z   = mesa_z + beer_h / 2
    approach_z = center_z + 0.15
    grasp_z    = center_z + margin
    lift_z     = approach_z + 0.20

    # ------------------ SEQUÊNCIA DE MOVIMENTOS ----------------
    # 1) Aproxima por cima
    pose_goal = arm.get_current_pose().pose
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = approach_z
    arm.set_pose_target(pose_goal)
    arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    # 2) Desce para grasp
    pose_goal.position.z = grasp_z
    arm.set_pose_target(pose_goal)
    arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    # 3) Fecha a garra e anexa
    close_gripper(gripper_client)
    rospy.sleep(0.8)      # garante contato estável
    attach_beer()
    rospy.sleep(1.0)

    # 4) Levanta com configuração segura
    pose_goal.position.z = lift_z
    arm.set_pose_target(pose_goal)
    arm.set_goal_tolerance(0.01)
    arm.set_planning_time(5.0)
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(0.5)
    success = arm.go(wait=True)
    arm.stop(); arm.clear_pose_targets()

    if not success:
        rospy.logwarn("Movimento de elevação falhou.")

    # 5) Move lateralmente (direita) para posicionar a beer
    pose_goal = arm.get_current_pose().pose
    pose_goal.position.y += 0.30  # desloca para o lado direito
    pose_goal.position.z += 0.02  # leve subida para evitar colisões no trajeto
    arm.set_pose_target(pose_goal)
    arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    # 6) Desce um pouco para apoiar a beer na nova posição
    pose_goal.position.z -= 0.30  # desce 8cm
    arm.set_pose_target(pose_goal)
    arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    # Solta
    detach_beer()
    open_gripper(gripper_client)

    # 7) Retorna à posição HOME (valores das juntas)
    home_joints = [0.0, -pi/2, pi/2, -pi/2, -pi/2, 0.0]
    arm.set_joint_value_target(home_joints)
    arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    rospy.loginfo("Pick‑and‑place concluído.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

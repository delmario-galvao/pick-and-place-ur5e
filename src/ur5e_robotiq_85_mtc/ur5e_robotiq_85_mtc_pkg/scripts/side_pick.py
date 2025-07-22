#!/usr/bin/env python3
# lateral_pick.py – grasp lateral seguro da lata (beer)

import os, rospy, yaml, moveit_commander, actionlib
from math import pi
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from tf.transformations import quaternion_from_euler

# ───────── parâmetros gerais ────────────────────────────────────────────────
BEER_RADIUS = 0.035     # m
BEER_HEIGHT = 0.160     # m
ROBOT       = "robot"
FINGERS     = ["left_inner_finger", "right_inner_finger"]

# ───────── funções auxiliares ───────────────────────────────────────────────
def gripper(pos, client, effort=100.0):
    goal = GripperCommandGoal()
    goal.command.position = pos      # 0.0 = aberta / 0.8 = fechada
    goal.command.max_effort = effort
    client.send_goal(goal); client.wait_for_result()

def attach(detach=False):
    srv = '/link_attacher_node/detach' if detach else '/link_attacher_node/attach'
    rospy.wait_for_service(srv)
    caller = rospy.ServiceProxy(srv, Attach)
    for link in FINGERS:
        req = AttachRequest()
        req.model_name_1, req.link_name_1 = ROBOT, link
        req.model_name_2, req.link_name_2 = "beer", "link"
        caller(req)

def load_scene(yaml_path, scene):
    with open(yaml_path) as f:
        for obj in yaml.safe_load(f)["planning_scene_objects"]:
            pose = PoseStamped(); cfg = obj.get("pose", {})
            pose.header.frame_id = cfg.get("frame_id", "world")
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = cfg.get("position",[0,0,0])
            pose.pose.orientation.x, pose.pose.orientation.y, \
            pose.pose.orientation.z, pose.pose.orientation.w = cfg.get("orientation",[0,0,0,1])
            scene.add_box(obj["name"], pose, size=tuple(obj["dimensions"]))
    rospy.sleep(1.0)

# ───────── sequência principal ──────────────────────────────────────────────
def main():
    rospy.init_node("lateral_pick")
    moveit_commander.roscpp_initialize([])

    scene = moveit_commander.PlanningSceneInterface()
    arm   = moveit_commander.MoveGroupCommander("manipulator")

    grip_cli = actionlib.SimpleActionClient(
        "/robotiq_2f_85_gripper_controller/gripper_cmd",
        GripperCommandAction)
    grip_cli.wait_for_server()

    # carrega objetos
    yaml_path = os.path.expanduser(
        "~/ur5e_mtc_ws/src/ur5e_robotiq_85_mtc/ur5e_robotiq_85_mtc_pkg/config/mtc/pick_and_place.yaml")
    load_scene(yaml_path, scene)

    # ─── parâmetros da trajetória ───────────────────────────────────────────
    x, y, mesa_z = 0.50, 0.00, 0.10        # posição da lata
    # margens
    SIDE_SAFETY   = 0.08                   # 8 cm de folga lateral ao descer
    ADVANCE_DIST  = 0.07                   # avanço final até tocar (~7 cm)
    # alturas
    top_z   = mesa_z + BEER_HEIGHT + 0.02  # 2 cm acima do topo
    grasp_z = mesa_z + BEER_HEIGHT/2 - 0.05
    lift_z  = mesa_z + BEER_HEIGHT/2 + 0.18

    # orientação: dedos horizontais apontando para −Y
    qx,qy,qz,qw = quaternion_from_euler(-pi/2, 0, pi)

    pose = arm.get_current_pose().pose      # reutilizaremos esse objeto
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx,qy,qz,qw

    # ─── 1) aproxima alto e afastado ────────────────────────────────────────
    pose.position.x = x
    pose.position.y = y + BEER_RADIUS + SIDE_SAFETY
    pose.position.z = top_z
    arm.set_pose_target(pose); arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    # ─── 2) desce na mesma Y ────────────────────────────────────────────────
    pose.position.z = grasp_z
    arm.set_pose_target(pose); arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    # ─── 3) avança lateralmente até encostar ────────────────────────────────
    pose.position.y = y + BEER_RADIUS + ADVANCE_DIST
    arm.set_pose_target(pose); arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    # ─── 4) fecha garra e faz attach ────────────────────────────────────────
    gripper(0.20, grip_cli); rospy.sleep(0.4); attach()

    # ─── 5) levanta a lata ──────────────────────────────────────────────────
    pose.position.z = lift_z
    arm.set_pose_target(pose); arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    # ─── 6) leva 30 cm p/ +Y e desce 8 cm para depositar ────────────────────
    pose.position.y += 0.30
    pose.position.z -= 0.08
    arm.set_pose_target(pose); arm.go(wait=True); arm.stop(); arm.clear_pose_targets()

    # ─── 7) solta e abre garra ──────────────────────────────────────────────
    attach(detach=True); gripper(0.0, grip_cli)

    # ─── 8) retorna à pose home (valores explícitos) ────────────────────────
    home = [0.0, -pi/2, pi/2, -pi/2, -pi/2, 0.0]
    arm.set_joint_value_target(home); arm.go(wait=True); arm.stop()

    rospy.loginfo("Grasp lateral concluído.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

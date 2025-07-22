#!/usr/bin/env python3
"""
ur5e_home.py

Baseado no MoveIt! Move Group Python Interface Tutorial:
  http://docs.ros.org/en/noetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

Este nó:
 - Inicializa o moveit_commander e ROS node
 - Chama go_to_joint_state() para ir à pose inicial desejada
 - Mantém o robot até Ctrl-C

Autor: Você
"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

class Ur5eHomePlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5e_home_planner', anonymous=True)

        # Interfaces básicas
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        # Substitua “ur5e_manipulator” pelo nome do seu MoveIt group
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        # Parâmetros de tolerância
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.005)
        self.group.set_planning_time(5.0)

        rospy.loginfo("UR5e Home Planner inicializado.")

    def go_to_joint_state(self, joint_goal):
        """
        Define a meta de juntas e executa o movimento.
        joint_goal: lista de 6 valores em radianos [J1, J2, ..., J6]
        """
        # Garante que temos 6 juntas
        assert len(joint_goal) == 6, "Esperado 6 valores de juntas"

        self.group.set_joint_value_target(joint_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        return plan

    def shutdown(self):
        """Limpa o MoveIt!"""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("MoveIt! finalizado.")

def main():
    planner = Ur5eHomePlanner()

    # Pose “home” solicitada
    # J1=0, J2=-π/2, J3,J4,J5,J6=0
    home_joints = [0.0, -pi/2, pi/2, -pi/2, -pi/2, 0.0]

    rospy.loginfo("Movendo para a pose HOME...")
    success = planner.go_to_joint_state(home_joints)

    if success:
        rospy.loginfo("Pose HOME atingida com sucesso!")
    else:
        rospy.logerr("Falha ao mover para HOME.")

    rospy.spin()
    planner.shutdown()

if __name__ == '__main__':
    main()

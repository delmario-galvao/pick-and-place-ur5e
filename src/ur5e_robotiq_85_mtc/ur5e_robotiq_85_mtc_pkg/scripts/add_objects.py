# utils/add_objects.py
import yaml, rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def load_scene_from_yaml(yaml_file):
    scene = PlanningSceneInterface(synchronous=True)
    rospy.sleep(1.0)                         # espera serviço subir

    with open(yaml_file) as f:
        data = yaml.safe_load(f)["objects"]

    for obj in data:
        pose = PoseStamped()
        pose.header.frame_id  = obj["frame_id"]
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = obj["pose"]["position"]
        pose.pose.orientation.x, pose.pose.orientation.y, \
        pose.pose.orientation.z, pose.pose.orientation.w = obj["pose"]["orientation"]

        if obj["type"] == "BOX":
            scene.add_box(obj["name"], pose, size=obj["size"])
        # extensível a CYLINDER, MESH, etc.

if __name__ == "__main__":
    rospy.init_node("add_scene_objects")
    load_scene_from_yaml(rospy.get_param("~yaml_file"))

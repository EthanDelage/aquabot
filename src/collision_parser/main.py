import xml.etree.ElementTree as ET
import numpy as np
from pprint import pprint

WORLD_PATH = "../aquabot/aquabot_gz/models/aquabot_regatta_world/model.xml"
MODEL_PATH = "../aquabot/aquabot_gz/models/"
MODEL_FILENAME = "model.sdf"


def parse_world_model(world_obstacle_pos, pose, root):
    dae = get_dae_file_from_include(root)
    if dae is not None:
        world_obstacle_pos.append((dae[0] + pose, dae[1], dae[2]))
        return
    model = root.find("model")
    for include in model.findall("include"):
        uri = include.find("uri")
        include_pose = include.find("pose")
        obstacle_pos = np.array([float(include_pose) for include_pose in include_pose.text.split()])
        obstacle_path = MODEL_PATH + uri.text + '/' + MODEL_FILENAME
        obstacle_model = ET.parse(obstacle_path).getroot()
        parse_world_model(world_obstacle_pos, np.array(pose + obstacle_pos), obstacle_model)
        # obstacle_mesh_pos = parse_obstacle_model(obstacle_model)
        # for mesh in obstacle_mesh_pos:
        #     result = obstacle_pos + mesh
        #     world_obstacle_pos.append(result)
    return world_obstacle_pos


def get_dae_file_from_include(root):
    model = root.find("model")
    collision = find_tag(model, "collision")
    if collision is not None:
        uri = find_tag(collision, "uri")
        if uri is None or uri.text != "mesh/rock.dae":
            return None
        collision_pose = np.array([float(pos) for pos in collision.find("pose").text.split()])
        scale = np.array([float(pos) for pos in find_tag(collision, "scale").text.split()])
        return collision_pose, scale, uri.text
    return None


def find_tag(root, tag_name):
    if root.tag == tag_name:
        return root
    for child in root:
        tag = find_tag(child, tag_name)
        if tag is not None:
            return tag
    return None


def parse_obstacle_model(model):
    allowed_uri = ["aquabot_rock"]
    mesh_poses = []
    for include in model.findall("include"):
        uri = include.find("uri")
        if uri.text in allowed_uri:
            pose = include.find("pose")
            current_mesh_pose = np.array([float(pos) for pos in pose.text.split()])
            mesh_poses.append(current_mesh_pose)
    return mesh_poses


if __name__ == "__main__":
    model = ET.parse(WORLD_PATH).getroot()
    world_obstacle = []
    pose = np.array([0, 0, 0, 0, 0, 0])
    parse_world_model(world_obstacle, pose, model)
    pprint(world_obstacle)

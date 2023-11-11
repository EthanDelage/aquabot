import xml.etree.ElementTree as ET
import numpy as np
from pprint import pprint
from dae_parser import get_bounding_box_from_dae
import transformations as tf

WORLD_PATH = "../aquabot/aquabot_gz/models/aquabot_regatta_world/model.sdf"
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

# Fonction pour appliquer les rotations
def apply_rotations(x, y, z, roll, pitch, yaw):
    # Définir la matrice de rotation pour chaque axe
    R_roll = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    # Combiner les matrices de rotation pour obtenir la matrice de rotation totale
    R_total = np.dot(R_yaw, np.dot(R_pitch, R_roll))

    # Appliquer la rotation au vecteur [x, y, z]
    rotated_vector = np.dot(R_total, np.array([x, y, z]))
    return rotated_vector


def transformation_3D(point, scale, rotation, translation):
    """
    Applique une transformation 3D à un point.

    Arguments :
    point : np.array, point 3D à transformer
    scale : np.array, vecteur 3D de scaling
    rotation : np.array, vecteur 3D d'angles de rotation en radians
    translation : np.array, vecteur 3D de translation

    Returns :
    np.array, point transformé
    """
    rotation = np.array([0, 0, 0])
    # Matrice de scaling
    scale_matrix = np.diag(scale)

    # Matrice de rotation autour de l'axe x
    rotation_x = np.array([[1, 0, 0],
                           [0, np.cos(rotation[0]), -np.sin(rotation[0])],
                           [0, np.sin(rotation[0]), np.cos(rotation[0])]])

    # Matrice de rotation autour de l'axe y
    rotation_y = np.array([[np.cos(rotation[1]), 0, np.sin(rotation[1])],
                           [0, 1, 0],
                           [-np.sin(rotation[1]), 0, np.cos(rotation[1])]])

    # Matrice de rotation autour de l'axe z
    rotation_z = np.array([[np.cos(rotation[2]), -np.sin(rotation[2]), 0],
                           [np.sin(rotation[2]), np.cos(rotation[2]), 0],
                           [0, 0, 1]])

    # Matrice de rotation totale dans l'ordre x, y, z
    rotation_matrix = rotation_z.dot(rotation_y).dot(rotation_x)

    # Appliquer la transformation complète
    transformed_point = scale_matrix.dot(rotation_matrix).dot(point) + translation

    return transformed_point


def convert_mesh_point_to_map(point, translation, rotation, scale):
    # point = apply_rotations(point[0], point[1], point[2], rotation[0], rotation[1], rotation[2])
    # point *= scale
    # point += translation
    # Appliquez la rotation, la translation et la mise à l'échelle
    # transformation_matrix = tf.euler_matrix(*rotation, 'sxyz')
    # transformation_matrix[:3, 3] = translation
    # transformation_matrix[:3, :3] *= scale
    #
    # # Mettez à jour la balise <pose> avec la nouvelle position et orientation
    # new_pose_values = tf.euler_from_matrix(transformation_matrix, 'sxyz') + translation

    return transformation_3D(point, scale, rotation, translation)
    # return point


def convert_rock_bounding_box_to_map(dae_bounding_box, rock):
    point1 = convert_mesh_point_to_map(dae_bounding_box[0][0], rock[0][:3], rock[0][3:], rock[1])
    point1[2] = 0
    point2 = convert_mesh_point_to_map(dae_bounding_box[0][1], rock[0][:3], rock[0][3:], rock[1])
    point2[2] = 0
    point3 = np.array([point1[0], point2[1], 0])
    point4 = np.array([point2[0], point1[1], 0])
    return point1, point2, point3, point4


def convert_map_bounding_box_to_str(map_bounding_box):
    return f"{map_bounding_box[0][0]} {map_bounding_box[0][1]}, " \
           f"{map_bounding_box[1][0]} {map_bounding_box[1][1]}, " \
           f"{map_bounding_box[2][0]} {map_bounding_box[2][1]}, " \
           f"{map_bounding_box[3][0]} {map_bounding_box[3][1]}\n"


def add_rock_map_bounding_box_in_file(rocks):
    # rock_dae_box = get_bounding_box_from_dae("../aquabot/aquabot_gz/models/aquabot_rock/mesh/rock.dae")
    i = 0
    with open("rocks.txt", "w") as file:
        for rock in rocks:
            rock_dae_box = get_bounding_box_from_dae("../aquabot/aquabot_gz/models/aquabot_rock/mesh/rock.dae", rock[0][3:])
            # print(rock[0])
            # print(rock[1])
            conversion = convert_rock_bounding_box_to_map(rock_dae_box, rock)
            # print(conversion)
            # print_boat(i + 1, conversion[0][0], conversion[0][1])
            i += 1
            # print_boat(i + 1, conversion[1][0], conversion[1][1])
            i += 1
            # print_boat(i + 1, conversion[2][0], conversion[2][1])
            i += 1
            # print_boat(i + 1, conversion[3][0], conversion[3][1])
            i += 1
            file.write(convert_map_bounding_box_to_str(conversion))


def print_boat(index, x, y):
    print(f"""
    <actor name="ally_{index}">
      <skin>
        <filename>../models/aquabot_greenboat/mesh/greenboat.dae</filename>
      </skin>
      <script>
        <loop>true</loop>
        <delay_start>20</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="square">
          <waypoint>
            <time>0.000000</time>
            <pose>{x} {y} 0 0 0 0</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>
    """)


if __name__ == "__main__":
    model = ET.parse(WORLD_PATH).getroot()
    world_obstacle = []
    pose = np.array([0, 0, 0, 0, 0, 0])
    rocks = parse_world_model(world_obstacle, pose, model)
    lighthouse_rock = rocks[0]
    add_rock_map_bounding_box_in_file(rocks)


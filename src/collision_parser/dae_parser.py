import xml.etree.ElementTree as ET
import numpy as np

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


def get_bounding_box_from_dae(file, rotation):
    tree = ET.parse(file)
    root = tree.getroot()
    ns = {
        root.tag.split("}")[1]: root.tag.split("}")[0][1:]
    }
    lib_geometry = root.find("COLLADA:library_geometries", ns)
    # for child in lib_geometry:
    #     print(child.tag)
    geometries = lib_geometry.findall("COLLADA:geometry", ns)
    bounding_boxes = []
    for geometry in geometries:
        mesh = geometry.find("COLLADA:mesh", ns)
        positions = find_positions(mesh)
        positions_array = positions.find("COLLADA:float_array", ns)
        vertices = [float(vertex) for vertex in positions_array.text.strip().split()]
        vertices = [transformation_3D((vertices[i], vertices[i + 1], vertices[i + 2]), np.array([1, 1, 1]), rotation, np.array([0,0,0])) for i in range(0, len(vertices), 3)]
        vertices = np.array(vertices)
        min_coord = np.min(vertices, axis=0)
        max_coord = np.max(vertices, axis=0)
        bounding_boxes.append((min_coord, max_coord))
    return bounding_boxes


def find_positions(mesh):
    for pos in mesh:
        id = pos.attrib.get("id")
        if id is not None and "positions" in id:
            return pos
    return None



if __name__ == "__main__":
    bounding_box = get_bounding_box_from_dae("../aquabot/aquabot_gz/models/aquabot_rock/mesh/rock.dae")
    print(bounding_box)
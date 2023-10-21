import xml.etree.ElementTree as ET
import numpy as np

def get_bounding_box_from_dae(file):
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
        vertices = [(vertices[i], vertices[i + 1], vertices[i + 2]) for i in range(0, len(vertices), 3)]
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
    bounding_box = get_bounding_box_from_dae("../aquabot/aquabot_gz/models/aquabot_rock/mesh/rock.xml")
    print(bounding_box)
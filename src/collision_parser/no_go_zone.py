import uuid

def get_bounding_box(no_go_zone):
    radius = no_go_zone.get("radius")
    x = no_go_zone.get("x")
    y = no_go_zone.get("y")
    point1 = (x - radius, y + radius)
    point2 = (x + radius, y + radius)
    point3 = (x + radius, y - radius)
    point4 = (x - radius, y - radius)
    return point1, point2, point3, point4


def print_saplings(bounding_box):
    for point in bounding_box:
        sapling = f"""
        <include>
          <name>sapling_{uuid.uuid4()}</name>
          <pose>{point[0]} {point[1]} 0 0 0 0</pose>
          <uri>aquabot_sapling</uri>
        </include>
        """
        print(sapling)


def get_bounding_boxes(no_go_zones):
    for no_go_zone in no_go_zones:
        bounding_box = get_bounding_box(no_go_zone)
        print(bounding_box)
        # print_saplings(bounding_box)

def generate_rocks_file(bounding_boxes):
    with open("rocks.txt", "w") as file:
        for bounding_box in bounding_boxes:
            for point in bounding_box:
                file.write(f"{point[0]},{point[1]} ")
            file.write("\n")

if __name__ == "__main__":
    no_go_zones = [{"x": 120, "y": -50, "radius": 25}, # Lighthouse island
                   {"x": -152, "y": -6, "radius": 50},
                   {"x": 110, "y": 130, "radius": 50},
                   {"x": 12, "y": -102, "radius": 25},
                   {"x": 92, "y": 170, "radius": 25},
                   {"x": -92, "y": 176, "radius": 30},
                   {"x": -40, "y": 220, "radius": 30},
                   {"x": -44, "y": -95, "radius": 30},
                   {"x": -30, "y": -150, "radius": 30},
                   ]
    opti_bounding_boxes = [
        ((95, -25), (145, -25), (145, -75), (95, -75)),
        ((-202, 44), (-102, 44), (-102, -56), (-202, -56)),
        ((60, 180), (160, 180), (160, 80), (60, 80)),
        ((-13, -77), (37, -77), (37, -127), (-13, -127)),
        ((67, 195), (117, 195), (117, 145), (67, 145)),
        ((-122, 206), (-62, 206), (-62, 146), (-122, 146)),
        ((-70, 250), (-10, 250), (-10, 190), (-70, 190)),
        ((-74, -65), (-14, -65), (-14, -125), (-74, -125)),
        ((-60, -120), (0, -120), (0, -180), (-60, -180))
    ]
    # # print_saplings(opti_bounding_box[0])
    # get_bounding_boxes(no_go_zones)
    generate_rocks_file(opti_bounding_boxes)
import yaml
import argparse

parser = argparse.ArgumentParser(description='Generate wall configuration.')
parser.add_argument('--flag', type=int, required=True, help='Flag to determine the wall type')
args = parser.parse_args()
flag = args.flag

obstacles = []
radius = 0.04  # sphere radius 

#  -- Static obstacles --
# flag = 0: Simple wall (parallel to y-axis)
# flag = 1: Simple wall (parallel to x-axis)
# flag = 2: Wall with Hole
# flag = 3: Wall belike letter "I"
# flag = 4: Wall belike letter "T"

#  -- Dynamic obstacles --
# flag = 5: Dynamics Simple wall (parallel to y-axis)
# others not implemented yet

#  -- Run command --
#  python3 generate_wall.py --flag X 
#  X = 0, 1, 2, 3, 4, 5 to generate the wall type


#---------------------------------------------------------------------------
if flag == 0: #  Simple wall  (parallel to y-axis)
    # Vertical wall
    for i in range(9):  # x
        for j in range(11):  # z
            obstacles.append({
                "name": f"VerticalWall_Ball_{i}_{j}",
                "position": [0.1+i * 2 * radius, 0.2, j * 2 * radius + 0.08],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
#---------------------------------------------------------------------------
if flag == 1: #  Simple wall (parallel to x-axis)
    # Vertical wall
    for i in range(9):  # y
        for j in range(11):  # z
            obstacles.append({
                "name": f"VerticalWall_Ball_{i}_{j}",
                "position": [0.5, -0.32+i * 2 * radius,j * 2 * radius + 0.08],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
#---------------------------------------------------------------------------
elif flag == 2: # Wall with Hole
    for i in range(9):  # y
        for j in range(11):  # z
            # hole
            if 2 <= i < 7 and 3 <= j < 7:
                continue
            obstacles.append({
                "name": f"VerticalWall_Ball_{i}_{j}",
                "position": [0.5, -0.32+i * 2 * radius,j * 2 * radius + 0.08],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
#---------------------------------------------------------------------------
elif flag == 3: # Wall belike letter "I"
    for i in range(5):  # y
        for j in range(7):  # z
            obstacles.append({
                "name": f"VerticalWall_Ball_{i}_{j}",
                "position": [1+-i * 2 * radius, 0, j * 2 * radius + 0.08],  # x = -y, y = x
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
    for i in range(4):  # x
        for j in range(5):  # y
            obstacles.append({
                "name": f"TopWall_Ball_{i}_{j}",
                "position": [1+-j * 2 * radius, -0.12 + i * 2 * radius, 0.64],  # x = -y, y = x
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
    for i in range(4):  # x
        for j in range(5):  # y
            obstacles.append({
                "name": f"BottomWall_Ball_{i}_{j}",
                "position": [1-j * 2 * radius, -0.12 + i * 2 * radius, 0],  # x = -y, y = x
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
#---------------------------------------------------------------------------
elif flag == 4: # Wall belike letter "T"
    for i in range(5):  # y
        for j in range(7):  # z
            obstacles.append({
                "name": f"VerticalWall_Ball_{i}_{j}",
                "position": [1+-i * 2 * radius, 0, j * 2 * radius + 0.08],  # x = -y, y = x
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
    for i in range(4):  # x
        for j in range(5):  # y
            obstacles.append({
                "name": f"TopWall_Ball_{i}_{j}",
                "position": [1+-j * 2 * radius, -0.12 + i * 2 * radius, 0.64],  # x = -y, y = x
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })

if flag == 5: #  Dynamics Simple wall  (parallel to y-axis)
    # Vertical wall
    for i in range(9):  # x
        for j in range(11):  # z
            obstacles.append({
                "name": f"VerticalWall_Ball_{i}_{j}",
                "position": [0.1+i * 2 * radius, 0.2, j * 2 * radius + 0.08],
                "velocity": [0.0, 0.1, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })            

output = {"obstacles": obstacles}
with open("obstacles_T.yaml", "w") as file:
    yaml.dump(output, file, default_flow_style=False)

print("---Reload obstacles_T.yaml---")
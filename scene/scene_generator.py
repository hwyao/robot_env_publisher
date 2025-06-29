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

# Default starting config and goal

output = {
    "initial_configuration": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "goal": {
        "position": [0.20,0.32, 0.30],
        "orientation": [1.0, 0.0, 0.0, 0.0]
    },
    "obstacles": []
}

def make_obstacle(name, pos, vel, r=radius, ang_speed=0.0):
    return {
        "name": name,
        "description": {
            "type": "SPHERE",
            "dimensions": [r]
        },
        "position": pos,
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "velocity_w": [0.0, 0.0, ang_speed],
        "velocity_v": vel,
        "use_velocity_as_twist": True
    }

# --------------------------------------------------------------------
if flag == 0:  # wall along y-axis
    for i in range(9):
        for j in range(11):
            pos = [0.1 + i * 2 * radius, 0.2, j * 2 * radius + 0.08]
            vel = [0.0, 0.0, 0.0]
            output["obstacles"].append(make_obstacle(f"VerticalWall_Ball_{i}_{j}", pos, vel))


# --------------------------------------------------------------------

elif flag == 1:  # wall along x-axis
    for i in range(9):
        for j in range(11):
            pos = [0.5, -0.32 + i * 2 * radius, j * 2 * radius + 0.08]
            vel = [0.0, 0.0, 0.0]
            output["obstacles"].append(make_obstacle(f"VerticalWall_Ball_{i}_{j}", pos, vel))

# --------------------------------------------------------------------
elif flag == 2:  # wall with hole
    for i in range(3):
        for j in range(5):
            if (1 <= i < 2 and 2 <= j < 3) or j in [0, 4]:
                continue
            pos = [0.5, -0.16 + 2.5 * i * 2 * radius, 2 * j * 2.5 * radius]
            vel = [0.0, 0.0, 0.0]
            output["obstacles"].append(make_obstacle(f"VerticalWall_Ball_{i}_{j}", pos, vel, r=0.08))

# --------------------------------------------------------------------            

elif flag == 3:  # letter I wall
    for i in range(1):
        for j in range(4):
            pos = [0.50 - i * 2 * radius, 0, 0.2 + j * 2 * radius + 0.08]
            vel = [0.0, 0., 0.0]
            output["obstacles"].append(make_obstacle(f"VerticalWall_Ball_{i}_{j}", pos, vel))
    for j in range(5):
        pos_top = [0.50, 0.16 - j * 2 * radius, 0.2 + 0.40]
        pos_bot = [0.50, 0.16 - j * 2 * radius, 0.2 + 0]
        vel = [0.0, 0.0, 0.0]
        output["obstacles"].append(make_obstacle(f"TopWall_Ball_{0}_{j}", pos_top, vel))
        output["obstacles"].append(make_obstacle(f"BottomWall_Ball_{0}_{j}", pos_bot, vel))

# --------------------------------------------------------------------

elif flag == 4:  # letter T wall
    for i in range(5):
        for j in range(7):
            pos = [1 - i * 2 * radius, 0, j * 2 * radius + 0.08]
            vel = [0.0, 0.0, 0.0]
            output["obstacles"].append(make_obstacle(f"VerticalWall_Ball_{i}_{j}", pos, vel))
    for i in range(4):
        for j in range(5):
            pos = [1 - j * 2 * radius, -0.12 + i * 2 * radius, 0.64]
            vel = [0.0, 0.0, 0.0]
            output["obstacles"].append(make_obstacle(f"TopWall_Ball_{i}_{j}", pos, vel))

# --------------------------------------------------------------------

elif flag == 5:  # dynamic wall (y-axis)
    for i in range(9):
        for j in range(11):
            pos = [0.1 + i * 2 * radius, 0.2, j * 2 * radius + 0.08]
            vel = [0.0, 0.1, 0.0]
            output["obstacles"].append(make_obstacle(f"VerticalWall_Ball_{i}_{j}", pos, vel))

# --------------------------------------------------------------------

elif flag == 6:  # selected z-positions
    for i in range(5):
        for j in range(20):
            if j in [5, 9]:
                pos = [0.15, -0.32 + i * 2 * radius + 0.16, j * 2 * radius + 0.08]
                vel = [0.0, 0.0, 0.05]
                output["obstacles"].append(make_obstacle(f"VerticalWall_Ball_{i}_{j}", pos, vel))

# -------------------- Output YAML --------------------
elif flag == 8:  # Tetris Russian bar
    # Define the bar as a vertical structure
    z_level = 0.5  # Fixed z-level for the bar
    x_start = 0.2  # Starting x position
    y_start = 0.3  # Fixed y position

    # Add 4 obstacles to form the bar
    for i in range(4):  # 4 obstacles in a vertical line
        pos = [x_start, y_start, z_level + i * 2 * radius]  # Increment z for each obstacle
        vel = [0.0, -0.08, 0.0]  # Static obstacles
        output["obstacles"].append(make_obstacle(f"TetrisBar_Ball_{i}", pos, vel))

    z_level = 0.62  # Fixed z-level for the bar
    x_start = 0.3  # Starting x position
    y_start = 0.3  # Fixed y position

    # Add 4 obstacles to form the bar
    for i in range(4):  # 4 obstacles in a horizontal line
        pos = [x_start + i * 2 * radius, y_start, z_level]  # Increment x for each obstacle
        vel = [0.0, -0.08, 0.0]  # Static obstacles
        output["obstacles"].append(make_obstacle(f"TetrisBar_YFixed_Ball_{i}", pos, vel))    

    z_level = 0.2  # Fixed z-level for the bar
    x_start = 0.22  # Starting x position
    y_start = 0.3  # Fixed y position

    # Add 4 obstacles to form the bar
    for i in range(4):  # 4 obstacles in a horizontal line
        pos = [x_start + i * 2 * radius, y_start, z_level]  # Increment x for each obstacle
        vel = [0.0, -0.08, 0.0]  # Static obstacles
        output["obstacles"].append(make_obstacle(f"TetrisBar_YFixed_Ball_{i}", pos, vel))   

    z_level = 0.2  # Fixed z-level for the bar
    x_start = 0.14  # Fixed x position
    y_start = 0.3  # Starting y position

    # Add 4 obstacles to form the bar
    for i in range(4):  # 4 obstacles in a horizontal line
        pos = [x_start, y_start + i * 2 * radius, z_level]  # Increment y for each obstacle
        vel = [0.0, -0.08, 0.0]  # Static obstacles
        output["obstacles"].append(make_obstacle(f"TetrisBar_XFixed_Ball_{i}", pos, vel))

    # Define the bar as a vertical structure
    z_level = 0.5  # Fixed z-level for the bar
    x_start = 0.12  # Starting x position
    y_start = 0.3  # Fixed y position

    # Add 4 obstacles to form the bar
    for i in range(4):  # 4 obstacles in a vertical line
        pos = [x_start, y_start, z_level + i * 2 * radius]  # Increment z for each obstacle
        vel = [0.0, -0.08, 0.0]  # Static obstacles
        output["obstacles"].append(make_obstacle(f"TetrisBar_Ball_{i}", pos, vel))
    # Define the bar as a vertical structure
    z_level = 0.5  # Fixed z-level for the bar
    x_start = 0.04  # Starting x position
    y_start = 0.3  # Fixed y position

    # Add 4 obstacles to form the bar
    for i in range(4):  # 4 obstacles in a vertical line
        pos = [x_start, y_start, z_level + i * 2 * radius]  # Increment z for each obstacle
        vel = [0.0, -0.08, 0.0]  # Static obstacles
        output["obstacles"].append(make_obstacle(f"TetrisBar_Ball_{i}", pos, vel))     

    z_level = 0.28  # Fixed z-level for the bar
    x_start = 0.14  # Starting x position
    y_start = 0.46  # Fixed y position

    # Add 4 obstacles to form the bar
    for i in range(5):  # 4 obstacles in a horizontal line
        pos = [x_start + i * 2 * radius, y_start, z_level]  # Increment x for each obstacle
        vel = [0.0, -0.08, 0.0]  # Static obstacles
        output["obstacles"].append(make_obstacle(f"TetrisBar_YFixed_Ball_{i}", pos, vel))               
# -------------------- Output YAML --------------------
with open("figureA33.yaml", "w") as file:
    yaml.dump(output, file, default_flow_style=False, sort_keys=False)

print("figureA33.yaml")

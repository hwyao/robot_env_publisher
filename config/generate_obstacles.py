import yaml

obstacles = []
radius = 0.4

flag = 1
if flag == 1:
# 竖板：4x4，沿y和z排布
    for i in range(8):  # y方向
        for j in range(4):  # z方向
            obstacles.append({
                "name": f"VerticalWall_Ball_{i}_{j}",
                "position": [4.5, i * 0.5, -1 + j * 0.5],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })

    for i in range(8):  # y方向
        for j in range(4):  # z方向
            obstacles.append({
                "name": f"VerticalWall_Ball_{i}_{j}",
                "position": [4.5, -i * 0.5, -1 + j * 0.5],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })

    # 上板：4x4，沿x和y排布，z固定在2.5
    for i in range(4):  # x方向
        for j in range(8):  # y方向
            obstacles.append({
                "name": f"TopWall_Ball_{i}_{j}",
                "position": [4 + i * 0.5, j * 0.5, 2.2],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
    for i in range(4):  # x方向
        for j in range(8):  # y方向
            obstacles.append({
                "name": f"TopWall_Ball_{i}_{j}",
                "position": [4 + i * 0.5, -j * 0.5, 2.2],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
    # 下板：4x4，沿x和y排布，z固定在0.0
    for i in range(4):  # x方向
        for j in range(8):  # y方向
            obstacles.append({
                "name": f"BottomWall_Ball_{i}_{j}",
                "position": [4 + i * 0.5, j * 0.5, -1.5],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })
    for i in range(4):  # x方向
        for j in range(8):  # y方向
            obstacles.append({
                "name": f"BottomWall_Ball_{i}_{j}",
                "position": [4 + i * 0.5, -j * 0.5, -1.5],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.1
            })        
elif flag == 2:

    radius = 0.1  # 每个球的半径

    # 桌子参数
    table_width = 2.0  
    table_depth = 1.0   
    table_height = 2.0  
    shelf_count = 2     

    center_y = 0.8
    left_wall_y = center_y - table_width / 2.0  # 左墙y=4
    right_wall_y = center_y + table_width / 2.0 # 右墙y=6

    # 侧板 - 左侧 (y = 4.0)
    for z in range(int(table_height / (2 * radius))):
        for x in range(int(table_depth / (2 * radius))):
            obstacles.append({
                "name": f"LeftWall_Ball_{x}_{z}",
                "position": [5+x * 2 * radius, left_wall_y, z * 2 * radius-0.5],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.0
            })

    # 侧板 - 右侧 (y = 6.0)
    for z in range(int(table_height / (2 * radius))):
        for x in range(int(table_depth / (2 * radius))):
            obstacles.append({
                "name": f"RightWall_Ball_{x}_{z}",
                "position": [5+x * 2 * radius, right_wall_y, z * 2 * radius-0.5],
                "velocity": [0.0, 0.0, 0.0],
                "radius": radius,
                "is_dynamic": False,
                "angular_speed": 0.0
            })

    # 搁板（包含最顶层） - 水平的
    for shelf_id in range(shelf_count+1):
        z = shelf_id * table_height / shelf_count
        for x in range(int(table_depth / (2 * radius))):
            for y in range(int(table_width / (2 * radius))):
                obstacles.append({
                    "name": f"Shelf_{shelf_id}_Ball_{x}_{y}",
                    "position": [5+x * 2 * radius, left_wall_y + y * 2 * radius, z-0.5],
                    "velocity": [0.0, 0.0, 0.0],
                    "radius": radius,
                    "is_dynamic": False,
                    "angular_speed": 0.0
                })

# 保存为YAML文件
output = {"obstacles": obstacles}
with open("obstacles_T.yaml", "w") as file:
    yaml.dump(output, file, default_flow_style=False)

print(" Set in ：obstacles_T.yaml")

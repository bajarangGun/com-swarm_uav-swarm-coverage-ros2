import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_nodes(context, *args, **kwargs):
    # 1. Extract ALL parameters from terminal
    robot_count = int(LaunchConfiguration('rc').perform(context))
    ws_x = int(LaunchConfiguration('ws_x').perform(context))
    ws_y = int(LaunchConfiguration('ws_y').perform(context))
    cell_size = float(LaunchConfiguration('cell_size').perform(context))
    takeoff_alt = float(LaunchConfiguration('takeoff_alt').perform(context))
    comm_range = float(LaunchConfiguration('comm_range').perform(context))
    drone_locs = LaunchConfiguration('drone_locs').perform(context)

    # =========================================================
    # 2. DYNAMIC MAP GENERATOR: Overwrites ws_obs_robs.txt
    # =========================================================
    pkg_dir = get_package_share_directory('con_plan_exec_pkg')
    input_dir = os.path.join(pkg_dir, 'input')
    os.makedirs(input_dir, exist_ok=True)
    file_path = os.path.join(input_dir, 'ws_obs_robs.txt')

    # Parse location string "x1,y1; x2,y2; x3,y3"
    loc_list = [tuple(map(int, p.strip().split(','))) for p in drone_locs.split(';')]
    
    # Build empty grid (0.5 = unexplored)
    grid = [['0.5' for _ in range(ws_y)] for _ in range(ws_x)]
    
    # Place drones (1.0, 2.0, 3.0...) at their specific start coordinates
    for i, (dx, dy) in enumerate(loc_list):
        if i < robot_count and dx < ws_x and dy < ws_y:
            grid[dx][dy] = str(float(i + 1))
            
    # Write the new map to the text file instantly before nodes boot
    with open(file_path, 'w') as f:
        for row in grid:
            f.write(','.join(row) + ',\n')
    # =========================================================

    nodes = []

    # 3. Pass threshold (comm_range) to the Central Planner
    central_node = Node(
        package='con_plan_exec_pkg',
        executable='concpp_node',
        name='concpp_node',
        output='screen',
        parameters=[{
            'ws_x': ws_x,
            'ws_y': ws_y,
            'rc': robot_count,
            'comm_range': comm_range  # <-- This is what was missing!
        }]
    )
    nodes.append(central_node)

    # 4. Spawn Drones
    for i in range(robot_count):
        drone_node = Node(
            package='con_plan_exec_pkg',
            executable='robot_node',
            name=f'drone{i}',
            output='screen',
            parameters=[{
                'rid': i,
                'ws_x': ws_x,
                'ws_y': ws_y,
                'cell_size': cell_size,
                'takeoff_alt': takeoff_alt
            }]
        )
        nodes.append(drone_node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rc', default_value='3', description='Number of robots'),
        DeclareLaunchArgument('ws_x', default_value='20', description='Workspace X size'),
        DeclareLaunchArgument('ws_y', default_value='20', description='Workspace Y size'),
        DeclareLaunchArgument('drone_locs', default_value='0,0; 1,0; 2,0', description='Initial grid locations X,Y'),
        DeclareLaunchArgument('cell_size', default_value='5.0', description='Physical size of grid cell (m)'),
        DeclareLaunchArgument('takeoff_alt', default_value='5.0', description='Flight altitude (m)'),
        DeclareLaunchArgument('comm_range', default_value='6.0', description='Tether Threshold (cells)'),
        OpaqueFunction(function=generate_launch_nodes)
    ])

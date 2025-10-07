from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    path_tracker_dir = get_package_share_directory('path_tracker')
    
    # --- 1. Declare Launch Arguments ---
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # NEW: Argument for the waypoints file
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='waypoints1.yaml',
        description='Name of the waypoints file in the config directory (e.g., waypoints1.yaml)'
    )
    
    # --- 2. Create Launch Configurations ---
    
    waypoints_file = LaunchConfiguration('waypoints_file')

    # --- 3. Construct the Full Path to the Waypoints File ---
    
    # PathJoinSubstitution to build the full path for the node
    full_waypoints_path = PathJoinSubstitution(
        [path_tracker_dir, 'config', waypoints_file]
    )

    # TurtleBot3 model
    turtlebot3_model = 'burger'
    
    # Set TURTLEBOT3_MODEL environment variable
    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=turtlebot3_model
    )
    
    # Path to RViz config file
    rviz_config = os.path.join(path_tracker_dir, 'config', 'trajectory_tracking.rviz')
    
    # Gazebo world launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    
    # --- 4. Define Nodes ---
    
    # Path Smoother Node
    path_smoother_node = Node(
        package='path_tracker',
        executable='path_smoother_node',
        name='path_smoother_node',
        output='screen',
        parameters=[
            full_waypoints_path,  # Pass the constructed full path
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Trajectory Generator Node
    trajectory_generator_node = Node(
        package='path_tracker',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'v_max': 0.22},  # TurtleBot3 Burger max speed
            {'a_max': 0.2},
            {'a_max_decel': 0.3},
            {'dt': 0.1}
        ]
    )
    
    # Trajectory Tracker Node from Gazebo odom
    trajectory_tracker_node = Node(
        package='path_tracker',
        executable='trajectory_tracker_node',
        name='trajectory_tracker_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'lookahead_distance': 0.5},
            {'max_linear_vel': 0.22},
            {'max_angular_vel': 2.84},
            {'position_tolerance': 0.15},
            {'control_frequency': 20.0},
            {'k_v': 1.0},
            {'k_omega': 3.0},
            {'start_x': 0.0},
            {'start_y': 0.0},
            {'start_theta': 0.0},
            {'use_gazebo_odom': True}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom')
        ]
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # --- 5. Return the Launch Description ---
    
    return LaunchDescription([
        set_turtlebot3_model,
        
        # Declare all launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial x position'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial y position'
        ),
        waypoints_file_arg,
        
        # Add nodes and other launch actions
        gazebo,
        path_smoother_node,
        trajectory_generator_node,
        trajectory_tracker_node,
        rviz_node
    ])
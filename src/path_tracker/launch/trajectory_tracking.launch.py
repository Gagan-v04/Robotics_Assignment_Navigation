from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory, which is the root of the install space
    pkg_share_dir = get_package_share_directory('path_tracker')
    
    # --- 1. Declare Launch Arguments ---
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # The default value is also just the filename.
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='waypoints1.yaml',
        description='Name of the waypoints file in the config directory (e.g., waypoints1.yaml)'
    )

    # --- 2. Create Launch Configurations ---
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    waypoints_file = LaunchConfiguration('waypoints_file')
    
    # --- 3. Construct the Full Path to the Waypoints File ---
    
    # PathJoinSubstitution to combine the config directory path and the filename.
    full_waypoints_path = PathJoinSubstitution(
        [pkg_share_dir, 'config', waypoints_file]
    )
    
    # --- 4. Define Nodes ---
    
    # Path Smoother Node 
    path_smoother_node = Node(
        package='path_tracker',
        executable='path_smoother_node',
        name='path_smoother_node',
        output='screen',
        parameters=[
            full_waypoints_path, 
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
            {'v_max': 0.5},
            {'a_max': 0.3},
            {'a_max_decel': 0.4},
            {'dt': 0.1}
        ]
    )
    
    # Trajectory Tracker Node
    trajectory_tracker_node = Node(
        package='path_tracker',
        executable='trajectory_tracker_node',
        name='trajectory_tracker_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'lookahead_distance': 0.5},
            {'max_linear_vel': 0.5},
            {'max_angular_vel': 1.0},
            {'position_tolerance': 0.15},
            {'control_frequency': 50.0},
            {'k_v': 1.0},
            {'k_omega': 2.0},
            {'start_x': 0.0},
            {'start_y': 0.0},
            {'start_theta': 0.0}
        ]
    )
    
    # RViz Node
    rviz_config = os.path.join(pkg_share_dir, 'config', 'trajectory_tracking.rviz')
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
        use_sim_time_arg,
        waypoints_file_arg,
        
        path_smoother_node,
        trajectory_generator_node,
        trajectory_tracker_node,
        rviz_node
    ])
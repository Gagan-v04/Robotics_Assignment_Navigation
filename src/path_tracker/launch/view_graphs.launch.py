# In launch/view_graphs.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file starts rqt_plot to visualize key performance topics.
    It assumes the main simulation is already running in another terminal.
    """

    # Define the RQT Plot node with the desired topics
    rqt_plot_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        output='screen',
        arguments=[
            '/robot_odom/twist/twist/linear/x',  # Actual linear velocity
            '/cmd_vel/linear/x',                  # Commanded linear velocity
            '/robot_acceleration/data'            # Actual linear acceleration
        ]
    )

    return LaunchDescription([
        rqt_plot_node
    ])
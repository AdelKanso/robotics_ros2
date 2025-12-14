from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cartographer_config_dir = '/home/masters/turtlebot3_ws/src/turtlebot3/turtlebot3_cartographer/config'
    cartographer_config = 'turtlebot3_lds_2d.lua'

    return LaunchDescription([

        # --- Cartographer SLAM ---
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config
            ]
        ),

        # --- Occupancy Grid (use the correct executable!) ---
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}]
        ),

        # --- Your Pose Recorder Node ---
        Node(
            package='robot_mapping',
            executable='slam_pose_recorder',
            name='slam_pose_recorder',
            output='screen'
        )
    ])

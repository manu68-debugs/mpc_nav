import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_launch_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    mpc_nav_dir = get_package_share_directory('mpc_nav')
    
    world_path = os.path.join(mpc_nav_dir, 'worlds', 'obstacle_test.world')
    
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', [os.path.join(mpc_nav_dir, 'config', 'sim.rviz')]],
        parameters=[{'use_sim_time': True}]
    )
    
    mpc_tracker = Node(
        package='mpc_nav',
        executable='mpc_controller',
        parameters=[{
            'use_sim_time': True,
            'v_max': 0.5,
            'a_lat_max': 0.5,
            'a_lon_max': 0.3,
            'dt': 0.1,
            'N': 15,
            'd_safe': 0.9,
        }]
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot,
        rviz,
        mpc_tracker
    ])

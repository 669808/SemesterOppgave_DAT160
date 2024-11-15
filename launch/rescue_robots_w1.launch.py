import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from geometry_msgs.msg import PoseWithCovarianceStamped

from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'multi_robot_challenge_23'

    # Get the path to the world, map and rviz configuration file
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'dat160_w1.world')
    map_file_path = os.path.join(get_package_share_directory(package_name), 'maps', 'map_dat160_w1.yaml')
    rviz_config_file_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'model.rviz')
    aruco_recognition_launch_file = os.path.join(get_package_share_directory(package_name), 'launch', 'aruco_recognition.launch.py') #added aruco_recognition_launch file.

    # Namespace of each robot
    first_tb3 = 'tb3_0'
    second_tb3 = 'tb3_1'
    # Starting position in the gazebo world of each robot
    first_tb3_pos = ['0.0', '-1.0', '0.0']
    second_tb3_pos = ['0.0', '1.0', '0.0']
    #Starting orientation in the gazebo world of each robot
    first_tb3_yaw = '0.0'
    second_tb3_yaw = '0.0'

    # Declaring use_sim_time as a launch argument that can then be used in all launch files
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    # Get launch argument use_sim_time as a launch configuration object
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Starting Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file_path}.items()
    )

    # Starting Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{"yaml_filename": map_file_path, "topic_name": "map", "frame_id": "map"}],
        # remappings=remappings
    )
    
    # Starting a lifecycle manager that takes care of the map server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ["map_server"]}]
    )

    # Spawning the first robot
    tb3_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch'), '/spawn_robot.launch.py']),
        launch_arguments={
            'namespace': first_tb3,
            'x': first_tb3_pos[0],
            'y': first_tb3_pos[1],
            'yaw': first_tb3_yaw,
        }.items()
    )

    # Spawning the second robot 
    tb3_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch'), '/spawn_robot.launch.py']),
        launch_arguments={
            'namespace': second_tb3,
            'x': second_tb3_pos[0],
            'y': second_tb3_pos[1],
            'yaw': second_tb3_yaw,
        }.items()
    )

    scoring = Node(
         package='scoring',
         executable='scoring',
         name='scoring'
    )

    # Include the ArUco recognition launch file
    aruco_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_recognition_launch_file),
        launch_arguments={'namespace': first_tb3}.items(),  
    )

    # Starting rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    go_to_point_server_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='go_to_point_a_star',
        name='tb3_0_go_to_point_service',
        parameters=[{'use_sim_time': use_sim_time},
                    {'namespace': first_tb3}]
    )

    go_to_point_server_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='go_to_point_a_star',
        name='tb3_1_go_to_point_service',
        parameters=[{'use_sim_time': use_sim_time},
                    {'namespace': second_tb3}]
    )

    # Add this Node for the frontier-based search

    #Legge til frontier search node i launch filen
    frontier_search_node = Node(
        package='multi_robot_challenge_23',  
        executable='frontier_based_search',  
        name='frontier_based_search',
        output='screen',  
        parameters=[
             {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/map', '/map'),  
            ('/cmd_vel', '/tb3_0/cmd_vel'),  
        ]
    )


    return LaunchDescription([
        sim_time_arg,
        gazebo,
        map_server,
        lifecycle_manager,
        tb3_0,
        tb3_1,
        rviz_node,
        scoring,
        aruco_recognition,  
        go_to_point_server_tb3_0,
        go_to_point_server_tb3_1,
        frontier_search_node,
    ])
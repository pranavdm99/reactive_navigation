import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_reactive_nav = get_package_share_directory('reactive_nav')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Path to the custom world file
    world_path = os.path.join(pkg_reactive_nav, 'worlds', 'bug_world.world')
    
    # Set TurtleBot3 model
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model)

    # Set GAZEBO_MODEL_PATH to include the share directory so model:// reactive_nav works
    gazebo_model_path = os.path.dirname(pkg_reactive_nav)
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [gazebo_model_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )

    # Gazebo launch
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Replicating TurtleBot3 Gazebo launch pattern
    # It includes robot_state_publisher.launch.py and spawn_turtlebot3.launch.py
    # from the turtlebot3_gazebo package.
    
    launch_file_dir = os.path.join(pkg_turtlebot3_gazebo, 'launch')

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0'
        }.items()
    )

    # Path to the parameters file
    config_path = os.path.join(pkg_reactive_nav, 'config', 'parameters.yaml')

    # Autonomous Navigation Node
    auto_nav_node = Node(
        package='reactive_nav',
        executable='autonomous_nav_node',
        name='autonomous_nav_node',
        output='screen',
        parameters=[config_path]
    )

    return LaunchDescription([
        set_tb3_model,
        set_gazebo_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot,
        auto_nav_node
    ])

import os
from os import pathsep
  
from ament_index_python.packages import get_package_share_directory, get_package_prefix
 
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


  
def generate_launch_description():

    rover_gz_description = get_package_share_directory("rover_gz_description")
    rover_gz_description_prefix = get_package_prefix('rover_gz_description')
    use_sim_time = LaunchConfiguration('use_sim_time') 

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
        )

    model_path = os.path.join('rover_gz_description', 'models')
    model_path += pathsep + os.path.join(rover_gz_description_prefix, "share")
    
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    urdf_file_name = "rover.urdf"
    urdf = os.path.join(
        get_package_share_directory('rover_gz_description'),
        'urdf',
        urdf_file_name
    )

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,'robot_description': robot_description}]
    ) 

    # start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
    #     get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"
    # )))

    # start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
    #     get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"
    # )))

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
        '-s', 'libgazebo_ros_init.so'], output='screen',
        )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","rover","-topic","robot_description",],
        output="screen"
    )

    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    # )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        env_variable,
        # model_arg,
        spawn_robot,
        robot_state_publisher_node,
        # start_gazebo_server,
        # start_gazebo_client
        gazebo
        
        # rviz_node
    ])
 

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

    # Get the directory and prefix of the rover_gz_description package
    rover_gz_description = get_package_share_directory("rover_gz_description")
    rover_gz_description_prefix = get_package_prefix('rover_gz_description')

    # Create a LaunchConfiguration variable to determine if simulation time is used
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare a launch argument for use_sim_time, defaulting to 'true'
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
        )

    # Set the model path for Gazebo models, including the package directory and share directory
    model_path = os.path.join('rover_gz_description', 'models')
    model_path += pathsep + os.path.join(rover_gz_description_prefix, "share")
    
    # Set the GAZEBO_MODEL_PATH environment variable to the model path
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Specify the URDF file name and its full path
    urdf_file_name = "rover.urdf"
    urdf = os.path.join(
        get_package_share_directory('rover_gz_description'),
        'urdf',
        urdf_file_name
    )

    # Read the URDF file content
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # Create a node for the robot_state_publisher to publish the robot's state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,'robot_description': robot_description}]
    )

    # Create an ExecuteProcess action to start Gazebo with necessary ROS plugins
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
        '-s', 'libgazebo_ros_init.so'], output='screen',
        )

    # Create a node to spawn the robot entity in Gazebo using the robot_description
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","rover","-topic","robot_description",],
        output="screen"
    )

    # Return a LaunchDescription object containing all actions and nodes to be launched
    return LaunchDescription([
        declare_use_sim_time_cmd, # Declare the use_sim_time launch argument
        env_variable, # Set the GAZEBO_MODEL_PATH environment variable
        spawn_robot, # Spawn the robot in Gazebo
        robot_state_publisher_node, # Publish the robot's state
        gazebo # Start Gazebo with specified plugins
    ])
 

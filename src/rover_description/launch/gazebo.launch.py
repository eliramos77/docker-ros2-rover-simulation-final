import os
from os import pathsep
  
from ament_index_python.packages import get_package_share_directory, get_package_prefix
 
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

  
def generate_launch_description():

    rover_description = get_package_share_directory("rover_description")
    rover_description_prefix = get_package_prefix('rover_description')

    model_path = os.path.join('rover_description', 'models')
    model_path =+ pathsep + os.path.join(rover_description_prefix, "share")
    
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        rover_description, 'urdf', 'rover.urdf'
                                        ),
                                      description='Absolute path to robot urdf file')   

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    ) 

    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"
    )))

    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"
    )))

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spwan.entity.py",
        arguments=["-entity","rover","-topic","robot_description"],
        output="screen"
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher_node,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])
 
 
    # model_arg = DeclareLaunchArgument(name='model', description='Absolute path to robot urdf file')
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # use_sim_time = LaunchConfiguration('use_sim_time') 
    # package_name = 'rover_description'
    # pkg_share = FindPackageShare(package=package_name).find(package_name)
    # pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 

    # #world_file_path = 'world.world'
    # #world = LaunchConfiguration('world')
    # #world_path = os.path.join(pkg_share, 'worlds',  world_file_path)

    # declare_use_sim_time_cmd = DeclareLaunchArgument(
    #     name='use_sim_time',
    #     default_value='true',
    #     description='Use simulation (Gazebo) clock if true'
    #     )

    # robot_name_in_model = 'rover'

    # # Get URDF via xacro

    # urdf_file_name = 'rover.urdf'
    # urdf = os.path.join(
    #     get_package_share_directory('rover_description'),
    #     'urdf',
    #     urdf_file_name
    #     )
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

    # robot_description = {"robot_description": robot_desc}
 
 
    # #rivz2
    # rviz2 = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='log',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )
 
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters= [{'use_sim_time': use_sim_time, 'robot_description': robot_desc}] #[{'use_sim_time': use_sim_time, "robot_description": robot_description_content}],
    # )

    # start_joint_state_publisher_cmd = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     name='joint_state_publisher',
    # )
 

    # '''declare_world_cmd = DeclareLaunchArgument(
    #     name='world',
    #     default_value=world_path,
    #     description='Full path to the world model file to load'
    #     ) '''
 
    # #spawn the robot 
    # spawn = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=["-topic", "/robot_description", 
    #                 "-entity", robot_name_in_model,
    #                 "-x", '0.0',
    #                 "-y", '0.0',
    #                 "-z", '0.05',
    #                 "-Y", '0.0']
    # )


    # gazebo = ExecuteProcess(
    #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
    #     '-s', 'libgazebo_ros_init.so'], output='screen',
    #     )

     
#     return LaunchDescription([
#     declare_use_sim_time_cmd,
#     spawn,
#     start_joint_state_publisher_cmd, 
#     robot_state_publisher_node,
#     gazebo
# ])
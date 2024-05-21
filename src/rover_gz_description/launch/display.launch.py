import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the URDF file name and construct the full path to the URDF file
    urdf_file_name = 'rover.urdf'
    urdf = os.path.join(
        get_package_share_directory('rover_gz_description'),
        'urdf',
        urdf_file_name
    )

    # Read the URDF file content into the robot_desc variable
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Create a Node action for the robot_state_publisher
    robot_state_pblisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',  # Corrected the name to match the common convention
        output="screen",
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]  # Pass the URDF file as an argument
    )

    # Create a Node action for the joint_state_publisher
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    # Create a Node action for the joint_state_publisher_gui
    joint_state_publisher__gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # Create a Node action for RViz2, a visualization tool
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # Return a LaunchDescription containing all the launch actions and nodes
    return launch.LaunchDescription([
        # Declare a launch argument named 'gui' with a default value of 'True'
        # This argument controls whether to use the joint_state_publisher_gui
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),

        joint_state_publisher_node,  # Add the joint_state_publisher_node to the launch description
        joint_state_publisher__gui_node,  # Add the joint_state_publisher_gui_node to the launch description
        robot_state_pblisher_node,  # Add the robot_state_publisher_node to the launch description
        rviz_node  # Add the rviz_node to the launch description
    ])

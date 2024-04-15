import os
from platform import node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    #Get URDF via xacro
    robot_description_content = Command(
    [
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([FindPackageShare("svaya_description"), "urdf","JTS_embr.urdf"]),

    ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
    [
    FindPackageShare("svaya_control"),
    "config",
    "svaya_control_node.yaml",
    ]
    )
    hw_parameters = PathJoinSubstitution(
        [
            FindPackageShare("svaya_control"),
            "config",
            "hw_parameters_12kg.yaml",
        ]
    )

    control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_description, robot_controllers,hw_parameters],
    output="both",
    )

    param_node = Node(
    package="svaya_control",
    executable="svaya_control_node",
    parameters=[hw_parameters],
    )
    robot_state_pub_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],)


    robot_controller_spawner_pos = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller",  "--controller-manager", "/controller_manager", "--inactive"],)

    rviz_node =      Node(
    package='rviz2',
    namespace='',
    executable='rviz2',
    name='rviz2',
    arguments=['-d' + os.path.join(get_package_share_directory('svaya_control'), 'config', 'rviz_config.rviz')]
            )

    util_node = Node(
    package="util_nodes",

    executable="fk",
    output="both",
    )

    model_torq_node = Node(
    package="util_nodes",
    parameters=[hw_parameters],
    executable="torq",
    output="both",
   # prefix = ["gdb -ex run --args"],
    )

#    rqt_node = ExecuteProcess(
#        cmd=['ros2','run','rqt_plot','rqt_plot','/svaya/grav_torq/data[0]','/svaya/torq_sensor/data','/svaya/torq_error/data'],
#        output="both",
#    )


    nodes = [
    param_node,
    control_node,
    robot_state_pub_node,
    joint_state_broadcaster_spawner,
    robot_controller_spawner_pos,
#    rviz_node,

  #  model_torq_node,
    ]

    return LaunchDescription(nodes)

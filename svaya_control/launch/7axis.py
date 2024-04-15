import os
from platform import node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #Get URDF via xacro
    robot_description_content = Command(
    [
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([FindPackageShare("svaya_description"), "urdf","svaya_aiu_7.urdf.xacro"]),

    ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
    [
    FindPackageShare("svaya_control"),
    "config",
    "controllers.yaml",
    ]
    )

    hw_parameters = PathJoinSubstitution(
        [
            FindPackageShare("svaya_control"),
            "config",
            "hw_parameters.yaml"
        ]
    )

    robot_parameters = PathJoinSubstitution(
    [
    FindPackageShare("svaya_control"),
    "config",
    "safety_controller.yaml",
    ]
    )

    #    controller_parameters = PathJoinSubstitution(
    #    [
    #        FindPackageShare("svaya_controller_manager"),
    #        "config",
    #        "controller_manager.yaml",
    #    ]
    #)
    controller_manager_node = Node(
    package="svaya_controller_manager",
    executable="test_node",

    )



    control_node = Node(
        package="svaya_control",
        executable="svaya_control_node",
        parameters=[robot_description, robot_controllers, robot_parameters, hw_parameters],
#        output="both",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    robot_state_pub_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[robot_description],
    )

    joint_state_pub_node = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    name = "joint_state_publisher",
    parameters=[{'source_list' : ['/svaya/joint_states'], 'rate':30}],
    )

    util_node = Node(
    package="util_nodes",
    executable="fk",
    )

    joint_states_node = Node(
    package="svaya_motionplan",
    executable="joint_states_node",
    )

    joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
    #        output="screen",
    #        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    position_trajectory_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["position_trajectory_controller", "-c", "/controller_manager"],
    )

    gravity_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["gravity_controller", "-c", "/controller_manager", "--inactive"],
    )

    safet_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["safety_controller", "-c", "/controller_manager"],
    parameters=[robot_parameters],
    )

    brake_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["brake_controller", "-c", "/controller_manager", "--inactive"],
    )


    joint_planner_node = Node(
    package="svaya_motionplan_testing",
    executable="main",
    )


    controller_stopper_node = Node(
    package="svaya_controller_manager",
    executable="controller_stopper",
    )

    rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=['-d' + os.path.join(get_package_share_directory('svaya_control'), 'config', 'config.rviz')]
,
    #            output="screen",
    )

    nodes = [
    control_node,
    robot_state_pub_node,
    joint_state_broadcaster_spawner,
    position_trajectory_controller_spawner,
#    gravity_controller_spawner,
#    brake_controller_spawner,
    safet_controller_spawner,

    rviz,

        util_node,

    ]

    return LaunchDescription(nodes)

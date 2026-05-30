from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    hardware_type = LaunchConfiguration('hardware_type').perform(context)

    moveit_configs = (
        MoveItConfigsBuilder(
            robot_name='wxai',
            package_name='trossen_arm_moveit',
        )
        .robot_description(
            file_path=PathJoinSubstitution([
                FindPackageShare('trossen_arm_description'),
                'urdf',
                'wxai.urdf.xacro',
            ]).perform(context),
            mappings={
                'ros2_control_hardware_type': hardware_type
            }
        )
        .robot_description_semantic(file_path='config/wxai.srdf.xacro')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )

    # ── Parâmetros do servo_node ──────────────────────────────
    servo_params = PathJoinSubstitution([
        FindPackageShare('trossen_arm_moveit'),
        'config',
        'servo_params.yaml',
    ])

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        output='screen',
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            moveit_configs.joint_limits,
            servo_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    return [servo_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar relógio da simulação MuJoCo'
        ),
        DeclareLaunchArgument(
            'hardware_type',
            default_value='real',
            description='mujoco | real'
        ),
        OpaqueFunction(function=launch_setup),
    ])
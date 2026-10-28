from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('trossen_arm_moveit'), 'launch'])
    hardware_type = LaunchConfiguration('ros2_control_hardware_type')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'moveit.launch.py'])),
        launch_arguments={
            'arm_variant': LaunchConfiguration('arm_variant'),
            'ip_address': LaunchConfiguration('ip_address'),
            'ros2_control_hardware_type': hardware_type,
            'use_moveit_rviz': LaunchConfiguration('use_moveit_rviz'),
            'enable_cameras': LaunchConfiguration('enable_cameras'),
        }.items(),
    )

    # Braço único na raiz: o servo_node publica em /arm_controller/joint_trajectory,
    # o mesmo controlador que o moveit.launch.py sobe.
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'servo.launch.py'])),
        launch_arguments={
            'hardware_type': hardware_type,
            'arm': 'none',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_variant',
            default_value='base',
            choices=('base', 'follower'),
            description='End effector variant of the Trossen Arm.',
        ),
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.1.2',
            description='IP address of the robot.',
        ),
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='real',
            choices=('real', 'mock_components'),
            description='Use real or mocked hardware interface (shared by MoveIt and Servo).',
        ),
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="Launches RViz with MoveIt's RViz configuration.",
        ),
        DeclareLaunchArgument(
            'enable_cameras',
            default_value='false',
            choices=('true', 'false'),
            description='Add the wrist + environment D435i cameras and bring up their drivers.',
        ),
        moveit_launch,
        servo_launch,
    ])

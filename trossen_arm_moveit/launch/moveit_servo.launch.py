from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def bringup_after_arm_config(context, moveit_launch, servo_launch, sim_launch, sim_servo_launch):
    """Aplica Kp e tolerância da garra (wxai_teleop_config) antes de subir MoveIt e Servo.

    Essas configurações se perdem quando o braço desliga, e o driver só aceita uma
    conexão, então o config roda e desconecta antes do ros2_control conectar. Se ele
    falhar (braço desligado, IP errado), o launch encerra em vez de subir pela metade.

    Com sim:=true sobe a simulação MuJoCo no lugar do braço real (sem config).
    """
    if LaunchConfiguration('sim').perform(context) == 'true':
        return [sim_launch, sim_servo_launch]

    configure = LaunchConfiguration('configure_arm').perform(context) == 'true'
    real = LaunchConfiguration('ros2_control_hardware_type').perform(context) == 'real'
    if not (configure and real):
        return [moveit_launch, servo_launch]

    timeout = LaunchConfiguration('configure_arm_timeout').perform(context)
    arm_config = ExecuteProcess(
        # O driver não tem timeout de conexão próprio: com o braço desligado ficaria preso
        cmd=['timeout', timeout, 'ros2', 'run', 'wxai_teleop_config', 'wxai_teleop_config',
             LaunchConfiguration('ip_address')],
        name='wxai_teleop_config',
        output='screen',
    )

    def on_config_exit(event, _context):
        if event.returncode == 0:
            return [moveit_launch, servo_launch]
        reason = 'timeout' if event.returncode == 124 else f'código {event.returncode}'
        return [
            LogInfo(msg=f'wxai_teleop_config falhou ({reason}); o braço está ligado e no IP certo?'),
            EmitEvent(event=Shutdown(reason='wxai_teleop_config failed')),
        ]

    return [
        arm_config,
        RegisterEventHandler(OnProcessExit(target_action=arm_config, on_exit=on_config_exit)),
    ]


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

    # Simulação: MuJoCo (com câmeras via CameraPlugin) + Servo no relógio da simulação
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'wxai_mujoco_sim.launch.py'])),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_moveit_rviz'),
            'enable_cameras': LaunchConfiguration('enable_cameras'),
            'use_sim_time': 'true',
        }.items(),
    )
    sim_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'servo.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            choices=('true', 'false'),
            description='Run the MuJoCo simulation instead of the real arm '
                        '(ignores ip_address and ros2_control_hardware_type).',
        ),
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
        DeclareLaunchArgument(
            'configure_arm',
            default_value='true',
            choices=('true', 'false'),
            description='Run wxai_teleop_config (Kp + gripper tolerance) before MoveIt. '
                        'Only applies to real hardware.',
        ),
        DeclareLaunchArgument(
            'configure_arm_timeout',
            default_value='20',
            description='Seconds to wait for wxai_teleop_config before giving up.',
        ),
        OpaqueFunction(
            function=bringup_after_arm_config,
            args=[moveit_launch, servo_launch, sim_launch, sim_servo_launch],
        ),
    ])

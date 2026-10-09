import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Builder do MoveIt: Resolve os erros de OMPL e Kinematics automaticamente
    moveit_configs = (
        MoveItConfigsBuilder(
            robot_name='wxai',
            package_name='trossen_arm_moveit',
        )
        .robot_description(
            file_path=PathJoinSubstitution([
                FindPackageShare('trossen_arm_description'),
                'urdf',
                'wxai_mujoco.urdf.xacro', # Certifique-se que o nome do arquivo bate com o seu URDF
            ]).perform(context),
            mappings={
                'ros2_control_hardware_type': 'mujoco',
                'enable_cameras': LaunchConfiguration('enable_cameras').perform(context),
            }
        )
        .robot_description_semantic(file_path='config/wxai.srdf.xacro')
        .trajectory_execution(file_path='config/moveit_controllers.yaml', moveit_manage_controllers=True)
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )

    # Adiciona use_sim_time aos parâmetros do Move Group
    move_group_params = moveit_configs.to_dict()
    move_group_params['use_sim_time'] = use_sim_time

    # 2. Nó do Move Group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[move_group_params],
        output='screen',
    )

    # 3. Nó do MuJoCo ROS 2 Control (Substitui o controller_manager padrão)
    ros2_controllers_filepath = PathJoinSubstitution([
        FindPackageShare('trossen_arm_moveit'), 
        'config', 
        'ros2_controllers.yaml'
    ])

    mujoco_control_params = [
        moveit_configs.robot_description,
        ros2_controllers_filepath,
        {'use_sim_time': use_sim_time},
    ]
    # [cameras] block start
    # CameraPlugin só é carregado com câmeras habilitadas; sem ele nenhuma imagem é renderizada.
    if LaunchConfiguration('enable_cameras').perform(context).lower() == 'true':
        mujoco_control_params.append(PathJoinSubstitution([
            FindPackageShare('trossen_arm_moveit'),
            'config',
            'mujoco_plugins.yaml'
        ]))
    # [cameras] block end

    mujoco_control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        parameters=mujoco_control_params,
        output='screen',
    )

    # 4. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            moveit_configs.robot_description,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    # 5. RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('trossen_arm_moveit'), 
        'config', 
        'moveit.rviz'
    ])
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.planning_pipelines,
            moveit_configs.robot_description_kinematics,
            moveit_configs.joint_limits,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    # 6. Spawners (Gatilhos atrelados à inicialização do MuJoCo)
    controller_spawner_nodes = []
    for controller_name in ['arm_controller', 'gripper_controller', 'joint_state_broadcaster']:
        controller_spawner_nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[controller_name, '--controller-manager', '/controller_manager'],
                output='screen',
            )
        )

    # [cameras] block start
    cameras_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('trossen_arm_bringup'),
                'launch',
                'cameras.launch.py'
            ]),
        ),
        condition=IfCondition(LaunchConfiguration('enable_cameras')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    # [cameras] block end

    return [
        robot_state_publisher_node,
        mujoco_control_node,
        move_group_node,
        rviz_node,
        cameras_launch_include,
        # O RegisterEventHandler substitui os TimerActions para garantir que
        # os spawners só rodem DEPOIS que o MuJoCo estiver de fato rodando
        RegisterEventHandler(
            OnProcessStart(
                target_action=mujoco_control_node,
                on_start=controller_spawner_nodes,
            )
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Usar relógio da simulação MuJoCo'),
        # [cameras]
        DeclareLaunchArgument(
            'enable_cameras',
            default_value='true',
            description='Adiciona as câmeras D435i (pulso + ambiente) ao robot_description',
        ),
        OpaqueFunction(function=launch_setup)
    ])
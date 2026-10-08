# Copyright 2025 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Brings up the two D435i cameras (wrist + environment).

On real hardware (use_sim_time:=false) this starts one realsense2_camera_node
per physical camera. In simulation (use_sim_time:=true) it does nothing: the
MuJoCo scene already renders and publishes both cameras -- with matching
topic names and frame ids -- through wxai_ros2_control.xml and the <sensor>
overrides in wxai_mujoco.urdf.xacro. Downstream nodes can subscribe to
/camera/wrist_camera/... and /camera/env_camera/... without caring which
mode is active. (The /camera/ prefix isn't a namespace we chose -- with two
cameras running, realsense2_camera_node falls back to it regardless of the
camera_namespace parameter, so the sim side matches that instead of fighting
the driver's default.)

align_depth is forced on for the real driver so its depth topic/frame lines
up with MuJoCo's single-viewpoint RGB-D output (real D435i color and depth
sensors have a small physical baseline that sim doesn't model).

Neither source publishes a compressed color stream on its own: MuJoCo's
camera sensor writes plain sensor_msgs/Image, and this repo's `ros2 topic
list` dumps while bringing up the real cameras never showed a `.../
compressed` topic either, so realsense2_camera_node isn't advertising one
here (no ros-jazzy-compressed-image-transport, or its Image publishers
aren't image_transport-wrapped). This file adds explicit image_transport
`republish` nodes (raw -> compressed) for both color streams so `.../color/
image_raw/compressed` exists identically in sim and on real hardware --
useful for bandwidth-constrained viewers like rqt_image_view over a
network or foxglove.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    wrist_camera_serial_no = LaunchConfiguration('wrist_camera_serial_no')
    env_camera_serial_no = LaunchConfiguration('env_camera_serial_no')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'Use MuJoCo for the cameras instead of real hardware. When true, this launch '
                'file starts no nodes -- see wxai_mujoco.urdf.xacro / wxai_ros2_control.xml.'
            ),
        ),
        DeclareLaunchArgument(
            'wrist_camera_serial_no',
            default_value='243322072096',
            description='Serial number of the wrist-mounted D435i. Required on real hardware.',
        ),
        DeclareLaunchArgument(
            'env_camera_serial_no',
            default_value='944122073060',
            description='Serial number of the environment D435i. Required on real hardware.',
        ),
    ]

    def realsense_node(camera_name, serial_no):
        return Node(
            condition=UnlessCondition(use_sim_time),
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=camera_name,
            parameters=[{
                # camera_name drives the default frame ids (e.g. wrist_camera_color_optical_
                # frame), matching rs_d435i.urdf.xacro. Topics end up at /camera/<camera_name>/
                # ... : with two cameras running, realsense2_camera_node falls back to that
                # shared /camera/ prefix regardless of camera_namespace or an external launch
                # namespace= (both were tried; see the module docstring), so it isn't set here.
                'camera_name': camera_name,
                # Without value_type=str, launch_ros infers the parameter's YAML type from the
                # resolved string; a serial number that looks like a pure integer (e.g.
                # "243322072096") would get written as an int, and realsense2_camera_node
                # rejects that since it declares serial_no as a string parameter.
                'serial_no': ParameterValue(serial_no, value_type=str),
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                # Nothing in this stack consumes the raw IR images (only the depth computed
                # from them, which still streams normally): dropping these two video streams
                # cuts USB bandwidth, needed when both D435i end up sharing one USB3 hub.
                'enable_infra1': False,
                'enable_infra2': False,
                # Nothing in this stack consumes IMU data (see rs_d435i.urdf.xacro).
                'enable_gyro': False,
                'enable_accel': False,
                # Without global time, frames are stamped in the HARDWARE_CLOCK domain: ROS time
                # at the first frame plus elapsed camera-clock time, which drifts from the host
                # clock and jumps when the camera counter wraps. Global time keeps stamps on the
                # host clock so images stay in sync with joint_states and with each other.
                'depth_module.global_time_enabled': True,
                'rgb_camera.global_time_enabled': True,
            }],
            output='screen',
        )

    def compressed_republish_node(camera_name):
        raw_topic = f'/camera/{camera_name}/color/image_raw'
        return Node(
            package='image_transport',
            executable='republish',
            name=f'{camera_name}_color_compressed_republisher',
            # image_transport's republish node reads the transport pair from the
            # in_transport/out_transport parameters, not from argv -- passing them as
            # `arguments=['raw', 'compressed']` is silently ignored (out_transport stays
            # empty, so it never advertises a compressed publisher).
            parameters=[{
                'in_transport': 'raw',
                'out_transport': 'compressed',
            }],
            remappings=[
                ('in', raw_topic),
                ('out/compressed', f'{raw_topic}/compressed'),
            ],
            output='screen',
        )

    return LaunchDescription(declared_arguments + [
        realsense_node('wrist_camera', wrist_camera_serial_no),
        realsense_node('env_camera', env_camera_serial_no),
        compressed_republish_node('wrist_camera'),
        compressed_republish_node('env_camera'),
    ])

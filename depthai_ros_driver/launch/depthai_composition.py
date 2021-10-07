# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the depthai driver in composition mode"""

from ament_index_python.packages import get_package_share_directory

import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import Command

################################################################################
# Parameters
camera_name = "bw1097"
driver_pkg_dir = get_package_share_directory('depthai_ros_driver')

queue_size = 1
calibration_file = os.path.join(
  driver_pkg_dir, f'resources/calibration/{camera_name}/depthai.calib')
blob_file = os.path.join(driver_pkg_dir, 'resources/mobilenet_ssd.blob')
blob_file_config = os.path.join(driver_pkg_dir, 'resources/mobilenet-ssd.json')
force_usb2 = False
sync_video_meta = False
compute_bbox_depth = False
full_fov_nn = True
rgb_height = 1080
rgb_fps = 30
depth_height = 720
depth_fps = 30
shaves = 14
cmx_slices = 14
nn_engines = 2
use_gdb = False
stream_list = [
  'left', 'right', 'metaout', 'previewout', 'disparity',
  'disparity_color', 'depth', 'video']

xacro_path = os.path.join(driver_pkg_dir, f'urdf/{camera_name}.urdf.xacro')

# Possible streams:
# *'left' - left mono camera preview
# *'right' - right mono camera preview
# *'previewout' - preview of stream sent to the NN
# *'metaout' - CNN output tensors
# *'depth' - the raw depth map, disparity converted to real life distance
# *'disparity' - disparity map, the disparity between left and right cams, in pixels
# *'disparity_color' - disparity map colorized
# *'meta_d2h' - device metadata stream
# *'video' - H.264/H.265 encoded color camera frames
# *'jpegout' - JPEG encoded color camera frames
# *'object_tracker' - Object tracker results

################################################################################

def generate_launch_description():
    """Generate launch description with multiple components."""

    container = ComposableNodeContainer(
            name='composition_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver',
                    plugin='rr::DepthAIBaseRos2',
                    name='depthai_node',
                    parameters=[{
                        "camera_name": camera_name,
                        "calibration_file": calibration_file,
                        "blob_file": blob_file,
                        "blob_file_config": blob_file_config,
                        "stream_list": stream_list,
                        "sync_video_meta": sync_video_meta,
                        "compute_bbox_depth": compute_bbox_depth,
                        "full_fov_nn": full_fov_nn,
                        "force_usb2": force_usb2,
                        "depth_fps": depth_fps,
                        "depth_height": depth_height,
                        "rgb_fps": rgb_fps,
                        "rgb_height": rgb_height,
                        "shaves": shaves,
                        "cmx_slices": cmx_slices,
                        "nn_engines": nn_engines,
                        "queue_size": queue_size,
                    }]
                )
            ],
            output='screen',
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            "tf_prefix": "",
            'robot_description': Command([f'xacro {xacro_path}'])
        }]
    )

    return launch.LaunchDescription([container, robot_state_pub])

# -----------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com> and Li Dahua
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
#
#

# Example file for two Blackfly S cameras where the primary camera triggers
# the secondary via its 3.3V signaling interface.
#
# One of them creates a master controller, the other one a follower. The exposure
# parameters are determined by the master. This is a useful setup for e.g. a
# synchronized stereo camera.
#
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Camera list with serial numbers
camera_list = {
    'cam0': '25366346', # primary camera
    'cam1': '25366344', # secondary camera 
}

# Parameters shared by all cameras
shared_cam_parameters = {
    'debug': False,
    'quiet': False,
    'buffer_queue_size': 1,
    'compute_brightness': False,  # fixed exposure/gain, no controllers needed
    'pixel_format': 'BGR8',
    'exposure_auto': 'Off',
    'exposure_time': 25000,  # fixed exposure in usec
    'gain_auto': 'Off',
    'gain': 12.0,
    'balance_white_auto': 'Off',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
}

# Parameters for the primary camera
primary_cam_parameters = {
    **shared_cam_parameters,
    'trigger_mode': 'Off',
    'line1_selector': 'Line1',
    'line1_linemode': 'Output',
    'line2_v33enable': False,
    'frame_rate': 10.0,
    'frame_rate_enable': True,
}

# Parameters for the secondary camera
secondary_cam_parameters = {
    **shared_cam_parameters,
    'trigger_mode': 'On',
    'trigger_source': 'Line3',
    'trigger_selector': 'FrameStart',
    'trigger_activation': 'RisingEdge',
    'trigger_overlap': 'Off',
    'trigger_delay': 0.0,
    'frame_rate_enable': False,
}


def make_parameters(context):
    """Generate camera parameters for the driver node."""
    param_dir = LaunchConfig('camera_parameter_directory').perform(context)
    calib_dir = LaunchConfig('calibration_directory').perform(context)
    calib_url = f'file://{calib_dir}/'

    driver_parameters = {
        'cameras': list(camera_list.keys()),
        'ffmpeg_image_transport.encoding': 'hevc_nvenc',
    }

    # Generate camera parameters
    parameter_file_path = os.path.join(param_dir, 'blackfly_s.yaml')
    primary_cam_parameters['parameter_file'] = parameter_file_path
    secondary_cam_parameters['parameter_file'] = parameter_file_path
    for cam, serial in camera_list.items():
        if cam == 'cam0':
            cam_params = {cam + '.' + k: v for k, v in primary_cam_parameters.items()}
        else:
            cam_params = {cam + '.' + k: v for k, v in secondary_cam_parameters.items()}
        cam_params[cam + '.serial_number'] = serial
        cam_params[cam + '.camerainfo_url'] = calib_url + serial + '.yaml'
        cam_params[cam + '.frame_id'] = cam
        driver_parameters.update(cam_params)
    return driver_parameters


def launch_setup(context, *args, **kwargs):
    container = ComposableNodeContainer(
        name='cam_sync_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='spinnaker_synchronized_camera_driver',
                plugin='spinnaker_synchronized_camera_driver::SynchronizedCameraDriver',
                name='cam_sync',
                parameters=[make_parameters(context)],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return [container]


def generate_launch_description():
    return LaunchDescription(
        [
            LaunchArg(
                'camera_parameter_directory',
                default_value=PJoin([FindPackageShare('spinnaker_camera_driver'), 'config']),
                description='Root directory for camera parameter definitions',
            ),
            LaunchArg(
                'calibration_directory',
                default_value=['camera_calibrations'],
                description='Root directory for camera calibration files',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

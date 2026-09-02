import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_h265_stereonet')
    default_params = os.path.join(pkg_share, 'config', 'stereo_h265_params.yaml')
    default_model = os.path.join(
        pkg_share, 'model', 'dstereo_s100_320_640_352_v2.4.hbm'
    )
    default_calib = os.path.join(pkg_share, 'config', 'vita_calib.yaml')

    return LaunchDescription([
        # ---- launch arguments ------------------------------------------------
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the parameter YAML file',
        ),
        DeclareLaunchArgument(
            'left_topic',
            default_value='/image_left_raw/h265',
            description='Left camera H.265 topic',
        ),
        DeclareLaunchArgument(
            'right_topic',
            default_value='/image_right_raw/h265',
            description='Right camera H.265 topic',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='StereoNet S100 HBM model path',
        ),
        DeclareLaunchArgument(
            'calib_yaml_file',
            default_value=default_calib,
            description='Stereo camera calibration YAML path',
        ),

        # ---- node ------------------------------------------------------------
        Node(
            package='ros2_h265_stereonet',
            executable='stereo_h265_node',
            name='stereo_h265_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'left_topic': LaunchConfiguration('left_topic'),
                    'right_topic': LaunchConfiguration('right_topic'),
                    'model_path': LaunchConfiguration('model_path'),
                    'calib_yaml_file': LaunchConfiguration('calib_yaml_file'),
                },
            ],
        ),
    ])

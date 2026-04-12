"""Launch the depth estimator node.

Starts ``depth_estimator_node`` which subscribes to the overview camera's
compressed image topic, runs Depth Anything V2 inference, and publishes
a 32FC1 relative inverse depth map:

    Input:  /ba_overview_camera/image_raw/compressed
    Output: /ba_overview_camera/depth/image_raw

Parameters are loaded from ``config/depth_estimator.yaml`` in the
installed share directory of this package.

The node is launched via the venv Python interpreter so that torch and
transformers (installed in the venv) are on sys.path.  The venv path
can be overridden with the ``venv_python`` launch argument.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Default venv python — the symlink resolves to /usr/bin/python3 but
# importantly adds ~/venvs/ba_depth_node/lib/…/site-packages to sys.path.
_DEFAULT_VENV_PYTHON = os.path.expanduser(
    '~/venvs/ba_depth_node/bin/python3')


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution([
        FindPackageShare('ba_depth_node'),
        'config',
        'depth_estimator.yaml',
    ])

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the parameter YAML for the depth estimator node.',
    )

    venv_python_arg = DeclareLaunchArgument(
        'venv_python',
        default_value=_DEFAULT_VENV_PYTHON,
        description='Path to the venv Python interpreter.',
    )

    depth_node = Node(
        package='ba_depth_node',
        executable='depth_estimator_node',
        name='depth_estimator',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        prefix=[LaunchConfiguration('venv_python'), ' '],
    )

    return LaunchDescription([
        params_file_arg,
        venv_python_arg,
        depth_node,
    ])

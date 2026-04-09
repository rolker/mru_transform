import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition

from lifecycle_msgs.msg import Transition

_SHARE = get_package_share_directory('mru_transform')


def generate_launch_description():

    geoid_grid_arg = DeclareLaunchArgument(
        'geoid_grid',
        default_value=os.path.join(
            _SHARE, 'data', 'geoid', 'us_noaa_g2018u0.tif'),
        description='Path to PROJ geoid grid (.tif)',
    )

    vdatum_grid_dir_arg = DeclareLaunchArgument(
        'vdatum_grid_dir',
        default_value=os.path.join(_SHARE, 'data', 'vdatum'),
        description='Directory containing VDatum *_mllw.gtx grids',
    )

    return LaunchDescription([
        geoid_grid_arg,
        vdatum_grid_dir_arg,
        LifecycleNode(
            package='mru_transform',
            executable='chart_datum_node',
            name='chart_datum',
            namespace='',
            respawn=True,
            respawn_delay=2,
            emulate_tty=True,
            parameters=[{
                'geoid_grid': LaunchConfiguration('geoid_grid'),
                'vdatum_grid_dir': LaunchConfiguration('vdatum_grid_dir'),
            }],
        ),
        LifecycleTransition(
            lifecycle_node_names=(
                PythonExpression(
                    expression=[
                        '"',
                        LaunchConfiguration("ros_namespace", default=''),
                        '" + "/chart_datum"',
                    ],
                ),
            ),
            transition_ids=(
                Transition.TRANSITION_CONFIGURE,
                Transition.TRANSITION_ACTIVATE,
            ),
        ),
    ])

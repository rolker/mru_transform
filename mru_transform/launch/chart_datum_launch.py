import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition

from lifecycle_msgs.msg import Transition

# Vertical-datum grids are NOT shipped with this package (ADR-0010 D5/D6: they
# live wherever imports run, never in the navigation runtime). They are
# provisioned into the world tree by enc_updater's datum provisioner
# (s57_tools#37); see unh_marine_autonomy#288 for the canonical layout.
# Absent grids do not fail the node -- it logs, disables VDatum and still
# reaches `inactive`. They are NOT harmless to the output: the polygon/param
# chain only covers the boat where the deployment supplies a datum_config_path
# entry or lake_datum. With this file's own defaults (datum_config_path '',
# lake_datum unset) VDatum is the ONLY source, so absent grids mean no
# chart_datum TF is ever published. See the README's "What absent grids
# actually cost".
_WORLD_DATUM = os.path.expanduser('~/data/world/datum')


def generate_launch_description():

    geoid_grid_arg = DeclareLaunchArgument(
        'geoid_grid',
        default_value=os.path.join(
            _WORLD_DATUM, 'geoid', 'us_noaa_g2018u0.tif'),
        description='Path to PROJ geoid grid (.tif). Default is the '
                    'world-tree location provisioned by enc_updater.',
    )

    vdatum_grid_dir_arg = DeclareLaunchArgument(
        'vdatum_grid_dir',
        default_value=os.path.join(_WORLD_DATUM, 'vdatum'),
        description='Directory containing VDatum *_mllw.gtx grids '
                    '(empty disables VDatum). Default is the world-tree '
                    'location provisioned by enc_updater.',
    )

    datum_config_path_arg = DeclareLaunchArgument(
        'datum_config_path',
        default_value='',
        description='Path to a polygon→datum YAML config (see '
                    'config/datum_polygons.example.yaml). Empty = none. '
                    'Deployment-specific polygons belong in the platform/site '
                    'config, not this package.',
    )

    return LaunchDescription([
        geoid_grid_arg,
        vdatum_grid_dir_arg,
        datum_config_path_arg,
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
                'datum_config_path': LaunchConfiguration('datum_config_path'),
            }],
            # The fixed-value override `lake_datum` (and `lake_datum_mhhw`) are
            # node parameters; set them via a param file or `--ros-args -p`
            # for a quick one-off (NaN/unset by default).
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

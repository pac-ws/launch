import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node_mission_control = Node(
            package='gcs',
            executable='mission_control',
            name='mission_control',
            output='screen',
            )
    mission_origin_params_file = '/workspace/launch/mission_origin_params.yaml'
    mission_origin_params = {}
    if os.path.exists(mission_origin_params_file):
        with open(mission_origin_params_file, 'r') as file:
            mission_origin_params = yaml.safe_load(file)
    else:
        print('Mission origin params file not found')
        exit(1)

    print('Mission origin params: ', mission_origin_params)

    node_mission_origin = Node(
            package='gcs',
            executable='mission_origin_gps',
            name='mission_origin_gps',
            namespace='pac_gcs',
            output='screen',
            parameters=[mission_origin_params],
            )

    node_rqt = Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            output='screen',
            arguments=['-ht', '-l', '-f', '--perspective-file', '/workspace/launch/mission_control.perspective'],
            )

    launch_description = LaunchDescription([node_mission_control, node_mission_origin, node_rqt])
    return launch_description

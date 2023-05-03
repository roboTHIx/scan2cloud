from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import math


def generate_launch_description():
  ld = LaunchDescription()


  scan2cloud = LaunchDescription([
        Node(
            package="scan2cloud",
            executable="scan2cloud_node",
            name="scan2cloud_node",
            namespace="",
            output="screen",
            parameters=[{
                "qos_cloud_pub": "default", #or default
                "qos_scan_sub": "default",  #or default
            }],
            remappings=[
                #from, to
                ("scan", "/sick/scan"),
                ("cloud", "cloud"),
            ],
        )
    ])

  ld.add_action(scan2cloud)


  return ld
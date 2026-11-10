"""Onboard task2 bringup; propulsion remains disabled by default."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
 return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('robot'),'launch','agx_bringup.launch.py'])),launch_arguments={'task':'task2'}.items())])

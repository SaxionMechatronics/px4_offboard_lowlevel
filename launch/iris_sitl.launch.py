import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config_1 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'uav_parameters',
      'iris_param.yaml'
      )
   
   config_2 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'sitl',
      'sitl_params.yaml'
      )

   config_3 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'controller',
      'initial_gains_iris.yaml'
      )
   
   return LaunchDescription([
      Node(
         package='px4_offboard_lowlevel',
         executable='offboard_controller_node',
         name='offboard_controller',
         parameters=[config_1, config_2, config_3]
      )
   ])
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config_1 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'uav_parameters',
      'x500_exp_param.yaml'
      )
   
   config_2 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'exp',
      'exp_params.yaml'
      )

   config_3 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'controller',
      'initial_gains_x500.yaml'
      )

   config_4 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'policy',
      'x500_policy.yaml'
      )
   
   return LaunchDescription([
      Node(
         package='px4_offboard_lowlevel',
         executable='offboard_controller_node',
         name='offboard_controller',
         parameters=[config_1, config_2, config_3, config_4],
         output='screen'
      )
   ])
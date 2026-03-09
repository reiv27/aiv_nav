#!/usr/bin/env python3
# Copyright 2024 reactive_circumnav
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  pkg_share = get_package_share_directory('reactive_circumnav')
  installed_config = os.path.join(pkg_share, 'config', 'controller_config.yaml')

  # Explicit path to config in source tree (used without rebuild)
  source_config = '/home/user/workspace/src/reactive_circumnav/config/controller_config.yaml'
  if os.path.isfile(source_config):
    default_params = source_config
  else:
    default_params = installed_config

  return LaunchDescription([
    DeclareLaunchArgument(
      'params_file',
      default_value=default_params,
      description='Path to controller_config.yaml',
    ),
    Node(
      package='reactive_circumnav',
      executable='reactive_circumnav',
      name='reactive_circumnav',
      parameters=[LaunchConfiguration('params_file')],
      output='screen',
    ),
  ])
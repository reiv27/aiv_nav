#!/usr/bin/env python3
# Copyright 2024 reactive_circumnav
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  pkg_share = get_package_share_directory('reactive_circumnav')
  config_path = os.path.join(pkg_share, 'config', 'controller_config.yaml')

  node = Node(
    package='reactive_circumnav',
    executable='reactive_circumnav',
    name='reactive_circumnav',
    parameters=[config_path],
    output='screen',
  )

  return LaunchDescription([node])
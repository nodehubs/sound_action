# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


# 生成启动描述函数
def generate_launch_description():
    # 包含手部标志检测节点的启动描述
    hand_lmk_det_node = IncludeLaunchDescription(
        # 使用Python启动描述源来包含其他启动文件
        PythonLaunchDescriptionSource(
            os.path.join(
                # 获取hand_lmk_detection包的共享目录
                get_package_share_directory('hand_lmk_detection'),
                'launch/hand_lmk_detection.launch.py')),  # 手部标志检测的启动文件路径
        # 设置启动参数
        launch_arguments={
            'smart_topic': '/hobot_hand_gesture_detection',  # 手势检测结果的发布话题
            'hand_lmk_pub_topic': '/hobot_hand_lmk_detection'  # 手部标志检测结果的发布话题
        }.items()
    )

    # 手势识别算法节点
    hand_gesture_det_node = Node(
        package='hand_gesture_detection',  # 节点所属包的名称
        executable='hand_gesture_detection',  # 可执行文件名称
        output='screen',  # 输出设置为屏幕
        parameters=[
            {"ai_msg_pub_topic_name": "/hobot_hand_gesture_detection"},  # 发布手势检测结果的话题
            {"ai_msg_sub_topic_name": "/hobot_hand_lmk_detection"}  # 订阅手部标志检测结果的话题
        ],
        arguments=['--ros-args', '--log-level', 'warn']  # 设置ROS参数及日志级别
    )

    # 返回启动描述，包含两个节点
    return LaunchDescription([
        hand_lmk_det_node,  # 手部标志检测节点
        hand_gesture_det_node  # 手势识别节点
    ])


# 新增订阅节点
    hand_gesture_sub_node = Node(
        package='your_package_name',  # 替换为你的包名
        executable='hand_gesture_subscriber',  # 可执行文件名称
        output='screen'
    )

    return LaunchDescription([
        hand_lmk_det_node,
        hand_gesture_det_node,
        hand_gesture_sub_node  # 加入新节点
    ])
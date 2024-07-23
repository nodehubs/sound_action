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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'audio_config_path',
            default_value='./config',
            description='hobot audio config path'),
        DeclareLaunchArgument(
            'audio_pub_topic_name',
            default_value='/audio_smart',
            description='hobot audio publish topic name'),
        # 启动音频采集pkg
        Node(
            package='hobot_audio',
            executable='hobot_audio',
            output='screen',
            parameters=[
                {"config_path": LaunchConfiguration('audio_config_path')},
                {"audio_pub_topic_name": LaunchConfiguration(
                    'audio_pub_topic_name')}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )

    ])
#ps aux | grep hobot_audio
#kill进程
#cd /opt/tros/lib/hobot_audio/hrsc
# hobot_audio默认发布的智能语音消息话题名为：/audio_smart
#ros2 topic list



#srpi-config

#语音输出

#sudo su
#source /opt/tros/setup.bash
#ros2 run hobot_tts hobot_tts
#ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="hw:0,0"
#ros2 topic pub --once /tts_text std_msgs/msg/String "{data: ""你知道地平线吗？是的，我知道地平线。它是一条从地面延伸到天空的线，它定义了地面和天空之间的分界线。""}"

#查看设备
#arecord -l

#开风扇
# cd /home/sunrise/x3pi/temp_control_C/i2c_fan
# sudo ./fan
#大语言
#ros2 run hobot_llm hobot_llm_chat
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_msg.msg import SmartAudioData
from answers import get_response


class AudioSmartToTTSAndGesture(Node):
    def __init__(self):
        super().__init__('audio_smart_to_tts_and_gesture')
        
        # 订阅 /audio_smart 话题
        self.subscription = self.create_subscription(
            SmartAudioData,
            '/audio_smart',
            self.listener_callback,
            10)
        
        # 发布 /tts_text 话题
        self.tts_publisher = self.create_publisher(String, '/tts_text', 10)
        
        # 发布 /command_topic 话题
        self.command_publisher = self.create_publisher(String, '/command_topic', 10)

    def listener_callback(self, msg):
        cmd_word = msg.cmd_word
        if cmd_word:  # 如果cmd_word不为空
            response = get_response(cmd_word)
            
            # 发布到 /tts_text
            self.get_logger().info(f'Publishing to /tts_text: {response}')
            tts_msg = String()
            tts_msg.data = response
            self.tts_publisher.publish(tts_msg)
            
            if cmd_word == "取药":
            # 发布 "取药" 指令到 /command_topic
                self.get_logger().info('Publishing to /command_topic: 取药')
                command_msg = String()
                command_msg.data = '取药'
                self.command_publisher.publish(command_msg)
            elif cmd_word == "呼叫医生":
             # 发布 "呼叫医生" 指令到 /command_topic
                self.get_logger().info('Publishing to /command_topic: 呼叫医生')
                command_msg = String()
                command_msg.data = '呼叫医生'
                self.command_publisher.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioSmartToTTSAndGesture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

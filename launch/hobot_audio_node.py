#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_msg.msg import SmartAudioData
from answers import get_response
import  websockets.sync.client
import json

class AudioSmartToTTS(Node):
    def __init__(self):
        uri_to = "ws://101.32.194.5:8000/ws/message/test"
        self.websocket_to =  websockets.sync.client.connect(uri_to)
        super().__init__('audio_smart_to_tts')
        self.subscription = self.create_subscription(
            SmartAudioData,
            '/audio_smart',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, '/tts_text', 10)

    def listener_callback(self, msg):
        cmd_word = msg.cmd_word
        if cmd_word:  # 如果cmd_word不为空
            self.websocket_to.send(json.dumps({
                "role": "user",
                "context":f'{cmd_word}'
            }, ensure_ascii=False))
            
            cmd_word = get_response(cmd_word)

            self.get_logger().info(f'Publishing to /tts_text: {cmd_word}')
            self.websocket_to.send(json.dumps({
                "role": "robot",
                "context":f'{cmd_word}'
            }, ensure_ascii=False))
            tts_msg = String()
            tts_msg.data = cmd_word
            self.publisher.publish(tts_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AudioSmartToTTS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets  # Ensure you're using the correct message type
import time
import json
import threading
import websockets.sync.client

class FallDownDetectionProcessor(Node):

    def __init__(self):
        super().__init__('fall_down_detection_processor')
        self.uri_to = "ws://101.32.194.5:8000/ws/falldown/test"
        self.websocket_to = websockets.sync.client.connect(self.uri_to)
        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_falldown_detection',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/tts_text', 10)
        self.subscription  # Prevent unused variable warning
        self.last_fall_down_time = 0
        self.fall_down_detected = False
        self.timer_thread = None
        self.timer_lock = threading.Lock()

    def reset_timer(self):
        with self.timer_lock:
            if self.timer_thread and self.timer_thread.is_alive():
                self.timer_thread.cancel()
            self.timer_thread = threading.Timer(5, self.send_false_if_no_fall_down)
            self.timer_thread.start()

    def send_false_if_no_fall_down(self):
        with self.timer_lock:
            if not self.fall_down_detected:
                self.websocket_to.send(json.dumps({
                    "fall_down": False
                }))
                self.get_logger().info('Sent: false after 5 seconds')
            self.fall_down_detected = False

    def listener_callback(self, msg):
        current_time = time.time()
        fall_detected = False
        for target in msg.targets:
            for attribute in target.attributes:
                if attribute.type == 'falldown' and attribute.value == 1.0:
                    fall_detected = True
                    break

        if fall_detected:
            if not self.fall_down_detected:
                self.fall_down_detected = True
                self.websocket_to.send(json.dumps({
                    "fall_down": True
                }))
                self.get_logger().info('Sent: true')
                
                tts_msg = String()
                tts_msg.data = "跌倒警报，请注意"
                self.publisher.publish(tts_msg)
                self.get_logger().info('Published: 跌倒警报，请注意')

            self.reset_timer()
        elif not fall_detected and self.fall_down_detected:
            self.get_logger().info('No fall detected, waiting to send false message.')

def main(args=None):
    rclpy.init(args=args)
    fall_down_detection_processor = FallDownDetectionProcessor()
    rclpy.spin(fall_down_detection_processor)
    fall_down_detection_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

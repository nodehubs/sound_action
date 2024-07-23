import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets, Attribute
import websockets.sync.client
import json
import threading

class GestureDetectionListener(Node):
    def __init__(self):
        uri_to = "ws://101.32.194.5:8000/ws/gesture/test"
        self.websocket_to = websockets.sync.client.connect(uri_to)
        super().__init__('gesture_detection_listener')
        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_hand_gesture_detection',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/gesture_text', 10)
        self.tts_publisher = self.create_publisher(String, '/tts_text', 10)
        self.last_gesture_value = None
        self.reset_timer = None

    def listener_callback(self, msg):
        for target in msg.targets:
            for attr in target.attributes:
                if attr.type == 'gesture':
                    gesture_value = attr.value

                    # Check if it's a known gesture and different from the last one
                    if gesture_value and gesture_value != self.last_gesture_value:
                        gesture_text = self.convert_gesture_to_text(gesture_value)
                        if gesture_text == "未知手势":
                            continue

                        tts_text = f"收到{gesture_text}指令，马上执行"
                        
                        self.websocket_to.send(json.dumps({
                            "get_medicine": gesture_value == 3.0,
                            "get_meal": gesture_value == 5.0,
                            "call_doctor": gesture_value == 11.0
                        }, ensure_ascii=False))

                        # Publish gesture text to /gesture_text topic
                        tts_msg = String()
                        tts_msg.data = gesture_text
                        self.publisher.publish(tts_msg)
                        self.get_logger().info(f'Gesture text: {gesture_text}')

                        # Publish formatted gesture text to /tts_text topic
                        tts_once_msg = String()
                        tts_once_msg.data = tts_text
                        self.tts_publisher.publish(tts_once_msg)

                        # Update last gesture value
                        self.last_gesture_value = gesture_value

                        # Cancel any existing reset timer
                        if self.reset_timer:
                            self.reset_timer.cancel()

                        # Set a timer to reset the gesture values after 5 seconds
                        self.reset_timer = threading.Timer(5.0, self.reset_gesture_values)
                        self.reset_timer.start()

    def convert_gesture_to_text(self, gesture_value):
        gesture_dict = {
            3.0: "取药",              # V手势-->取药
            5.0: "取餐",              # 手掌-->取餐
            11.0: "呼叫医生"          # OK手势-->呼叫医生
        }
        return gesture_dict.get(gesture_value, "未知手势")

    def reset_gesture_values(self):
        self.websocket_to.send(json.dumps({
            "get_medicine": False,
            "get_meal": False,
            "call_doctor": False
        }, ensure_ascii=False))
        self.get_logger().info('Reset gesture values sent')

def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

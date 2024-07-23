import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets, Attribute
import threading
import time

class GestureDetectionListener(Node):
    def __init__(self):
        super().__init__('gesture_detection_listener')
        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_hand_gesture_detection',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/gesture_text', 10)
        self.tts_publisher = self.create_publisher(String, '/tts_text', 10)
        self.command_publisher = self.create_publisher(String, '/command_topic', 10)

        self.last_transmit_time = None
        self.reset_timer = None

    def listener_callback(self, msg):
        for target in msg.targets:
            for attr in target.attributes:
                if attr.type == 'gesture':
                    gesture_value = attr.value

                    # Check if it's a known gesture
                    if gesture_value:
                        current_time = time.time()

                        # Check if it's within the cooldown period (3 seconds)
                        if self.last_transmit_time and current_time - self.last_transmit_time < 3.0:
                            continue  # Skip transmission

                        gesture_text = self.convert_gesture_to_text(gesture_value)
                        if gesture_text == "未知手势":
                            continue

                        tts_text = f"收到{gesture_text}指令，马上执行"

                        # Publish gesture text to /gesture_text topic
                        tts_msg = String()
                        tts_msg.data = gesture_text
                        self.publisher.publish(tts_msg)
                        self.get_logger().info(f'Gesture text: {gesture_text}')

                        # Publish formatted gesture text to /tts_text topic
                        tts_once_msg = String()
                        tts_once_msg.data = tts_text
                        self.tts_publisher.publish(tts_once_msg)

                        if gesture_text == "取药":
                        # 发布 "取药" 指令到 /command_topic
                            self.get_logger().info('Publishing to /command_topic: 取药')
                            command_msg = String()
                            command_msg.data = '取药'
                            self.command_publisher.publish(command_msg)
                        elif gesture_text == "呼叫医生":
                        # 发布 "呼叫医生" 指令到 /command_topic
                            self.get_logger().info('Publishing to /command_topic: 呼叫医生')
                            command_msg = String()
                            command_msg.data = '呼叫医生'
                            self.command_publisher.publish(command_msg)
                        
                        # Update transmit time
                        self.last_transmit_time = current_time

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
        self.get_logger().info('Reset gesture values')

def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

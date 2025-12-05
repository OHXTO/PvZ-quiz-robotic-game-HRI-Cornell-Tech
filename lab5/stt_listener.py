import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr


class STTListener(Node):
    def __init__(self):
        super().__init__('stt_listener')
        self.publisher_ = self.create_publisher(String, '/color_decision', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.get_logger().info("STT listener started. Say 'true' or 'false'.")
        self.timer = self.create_timer(3.0, self.listen_once)

    def listen_once(self):
        with self.microphone as source:
            self.get_logger().info("Listening...")
            audio = self.recognizer.listen(source, phrase_time_limit=3)

        try:
            text = self.recognizer.recognize_google(audio).lower()
            self.get_logger().info(f"Recognized: {text}")

            if "true" in text:
                self.publisher_.publish(String(data="true"))
                self.get_logger().info("Published: true")
            elif "false" in text:
                self.publisher_.publish(String(data="false"))
                self.get_logger().info("Published: false")
            else:
                self.get_logger().info("No valid keyword detected.")

        except sr.UnknownValueError:
            self.get_logger().info("Could not understand audio.")
        except sr.RequestError as e:
            self.get_logger().error(f"Speech recognition error: {e}")


def main():
    rclpy.init()
    node = STTListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
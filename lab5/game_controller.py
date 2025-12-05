# game_controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import subprocess
import random

AUDIO_DIR = '/home/ubuntu/ros2_ws/src/lab5/lab5/game_audio'

QUESTIONS = [
    ("All plants need soil to grow.", False, f"{AUDIO_DIR}/q0.wav"),
    ("Flowers are the reproductive organs of most plants.", True, f"{AUDIO_DIR}/q1.wav"),
    ("Plants make their own food through photosynthesis using sunlight, water, and carbon dioxide.", True, f"{AUDIO_DIR}/q2.wav"),
    ("Sunflowers can produce energy for other plants around them.", False, f"{AUDIO_DIR}/q3.wav"),
    ("Rainforests have the largest variety of plant species on Earth.", True, f"{AUDIO_DIR}/q4.wav"),
    ("All flowers bloom in spring.", False, f"{AUDIO_DIR}/q5.wav"),
    ("Banana plants are actually giant herbs, not trees.", True, f"{AUDIO_DIR}/q6.wav"),
    ("In Plants vs. Zombies, the Peashooter is based on a real pea plant.", True, f"{AUDIO_DIR}/q7.wav"),
    ("Carnivorous plants like the man-eating plant really exist on Earth.", False, f"{AUDIO_DIR}/q8.wav"),
    ("Cherry Bombs are based on a real plant that can explode.", False, f"{AUDIO_DIR}/q9.wav"),
    ("Carnivorous plants live in nutrient-rich soils.", False, f"{AUDIO_DIR}/q10.wav"),
    ("There is a real plant called the Snow Pea.", True, f"{AUDIO_DIR}/q11.wav"),
    ("Sunflowers always follow the sun, even after they bloom.", False, f"{AUDIO_DIR}/q12.wav"),
    ("Some flowers open only at night.", True, f"{AUDIO_DIR}/q13.wav"),
    ("Apples float in water because they are 25 percent air.", True, f"{AUDIO_DIR}/q14.wav"),
    ("Broccoli is actually a flower.", True, f"{AUDIO_DIR}/q15.wav"),
    ("Carrots used to be purple before they became orange.", True, f"{AUDIO_DIR}/q16.wav"),
    ("Tomatoes are fruits, not vegetables.", True, f"{AUDIO_DIR}/q17.wav"),
    ("The smell of freshly cut grass is actually a distress signal from the plant.", True, f"{AUDIO_DIR}/q18.wav"),
    ("Plants have both male and female parts in the same flower.", True, f"{AUDIO_DIR}/q19.wav"),
]

AUDIO = {
    'ready': f'{AUDIO_DIR}/ready.wav',
    'correct': f'{AUDIO_DIR}/correct.wav',
    'wrong': f'{AUDIO_DIR}/wrong.wav',
    'win': f'{AUDIO_DIR}/win.wav',
    'lose': f'{AUDIO_DIR}/lose.wav',
}


class GameController(Node):
    def __init__(self):
        super().__init__('game_controller')

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/answer_result", self.on_color_msg, 10)

        self.speed = 0.08
        self.min_speed = 0.08
        self.max_speed = 0.20
        self.speed_up = 0.03    
        self.speed_down = 0.02 

        self.required_correct = 4
        self.total_questions = 20
        self.track_length = 3.8
        self.current_distance = 0.0

        self.questions = QUESTIONS[:self.total_questions]
        random.shuffle(self.questions)
        self.current_q_index = 0
        self.correct_count = 0
        self.waiting_answer = False
        self.game_over = False

        self.create_timer(0.1, self.publish_velocity)
        self.create_timer(0.1, self.update_distance)

        self.play_audio(AUDIO['ready'])
        self.call_after(3.0, self.ask_current_question)

    def call_after(self, delay, func):
        def wrapper():
            func()
            timer.cancel()
        timer = self.create_timer(delay, wrapper)

    def play_audio(self, wav_file):
        try:
            subprocess.Popen(['aplay', '-D', 'plughw:0,0', wav_file],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except:
            pass

    def ask_current_question(self):
        if self.game_over:
            return

        if self.current_q_index >= self.total_questions:
            self.finish_game()
            return

        _, _, wav = self.questions[self.current_q_index]
        self.waiting_answer = True
        self.play_audio(wav)

    def finish_game(self):
        if self.correct_count >= self.required_correct:
            self.play_audio(AUDIO['win'])
        else:
            self.play_audio(AUDIO['lose'])
        self.stop_robot()
        self.game_over = True

    def normalize(self, s):
        v = s.strip().lower()
        if v in ["true", "green"]:
            return True
        if v in ["false", "red"]:
            return False
        return None

    def on_color_msg(self, msg):
        if self.game_over or not self.waiting_answer:
            return

        answer = self.normalize(msg.data)
        if answer is None:
            return

        _, correct, _ = self.questions[self.current_q_index]
        is_correct = (answer == correct)

        self.waiting_answer = False
        self.current_q_index += 1

        if is_correct:
            self.correct_count += 1
            self.speed = max(self.min_speed, self.speed - self.speed_down)
            self.play_audio(AUDIO['correct'])
        else:
            self.speed = min(self.max_speed, self.speed + self.speed_up)
            self.play_audio(AUDIO['wrong'])

        if self.correct_count >= self.required_correct:
            self.call_after(1.0, lambda: self.play_audio(AUDIO['win']))
            self.stop_robot()
            self.game_over = True
        else:
            self.call_after(1.0, self.ask_current_question)

    def update_distance(self):
        if self.game_over:
            return

        self.current_distance += self.speed * 0.1

        if self.current_distance >= self.track_length:
            self.play_audio(AUDIO['lose'])
            self.stop_robot()
            self.game_over = True

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.speed if not self.game_over else 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        self.speed = 0.0
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GameController()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

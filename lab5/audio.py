# pregenerate_audio.py

import subprocess
import os

audio_dir = '/home/ubuntu/ros2_ws/src/lab5/lab5/game_audio'
os.makedirs(audio_dir, exist_ok=True)

# 固定语音
phrases = {
    'ready': "Get ready! The zombie is coming!",
    'correct': "Correct",
    'wrong': "Wrong",
    'win': "You win! Zombie stopped!",
    'lose': "Zombie reached the brain! Game over!",
}

# 问题
questions = [
    "All plants need soil to grow.",
    "Flowers are the reproductive organs of most plants.",
    "Plants make their own food through photosynthesis using sunlight, water, and carbon dioxide.",
    "Sunflowers can produce energy for other plants around them.",
    "Rainforests have the largest variety of plant species on Earth.",
    "All flowers bloom in spring.",
    "Banana plants are actually giant herbs, not trees.",
    "In Plants vs. Zombies, the Peashooter is based on a real pea plant.",
    "Carnivorous plants like the man-eating plant really exist on Earth.",
    "Cherry Bombs are based on a real plant that can explode.",
    "Carnivorous plants live in nutrient-rich soils.",
    "There is a real plant called the Snow Pea.",
    "Sunflowers always follow the sun, even after they bloom.",
    "Some flowers open only at night.",
    "Apples float in water because they are 25 percent air.",
    "Broccoli is actually a flower.",
    "Carrots used to be purple before they became orange.",
    "Tomatoes are fruits, not vegetables.",
    "The smell of freshly cut grass is actually a distress signal from the plant.",
    "Plants have both male and female parts in the same flower.",
]

print("Generating fixed phrases...")
for name, text in phrases.items():
    print(f"  {name}")
    subprocess.run(
        f'echo "{text}" | piper -m ~/.local/share/piper/en_US-lessac-medium.onnx --length-scale 1.2 --output_file {audio_dir}/{name}.wav',
        shell=True
    )

print("Generating questions...")
for i, q in enumerate(questions):
    print(f"  q{i}: {q[:30]}...")
    subprocess.run(
        f'echo "{q}" | piper -m ~/.local/share/piper/en_US-lessac-medium.onnx --length-scale 1.2 --output_file {audio_dir}/q{i}.wav',
        shell=True
    )

print("Done!")
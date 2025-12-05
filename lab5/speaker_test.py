# test_piper.py

import subprocess

text = "Hello! The zombie is coming!"

# test_piper.py

import subprocess

text = "Hello! The zombie is coming!"

print("1. 正常速度")
subprocess.run(
    f'echo "{text}" | piper -m ~/.local/share/piper/en_US-lessac-medium.onnx --output_file /tmp/test.wav && aplay -D plughw:0,0 /tmp/test.wav',
    shell=True
)

print("2. 慢一点 (1.2)")
subprocess.run(
    f'echo "{text}" | piper -m ~/.local/share/piper/en_US-lessac-medium.onnx --length-scale 1.2 --output_file /tmp/test.wav && aplay -D plughw:0,0 /tmp/test.wav',
    shell=True
)

print("3. 更慢 (1.4)")
subprocess.run(
    f'echo "{text}" | piper -m ~/.local/share/piper/en_US-lessac-medium.onnx --length-scale 1.4 --output_file /tmp/test.wav && aplay -D plughw:0,0 /tmp/test.wav',
    shell=True
)
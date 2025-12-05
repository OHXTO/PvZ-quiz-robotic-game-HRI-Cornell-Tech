import speech_recognition as sr

r = sr.Recognizer()

# 列出所有可用麦克风
print("Available microphones:")
for i, name in enumerate(sr.Microphone.list_microphone_names()):
    print(f"{i}: {name}")

mic_index = int(input("Enter microphone index to use: "))

with sr.Microphone(device_index=mic_index) as source:
    print("Adjusting for ambient noise... please wait 1 second.")
    r.adjust_for_ambient_noise(source, duration=1)
    print("Say something (3 seconds limit)...")
    audio = r.listen(source, phrase_time_limit=3)

print("Recognizing...")
try:
    text = r.recognize_google(audio, language="en-US")
    print("You said:", text)
except sr.UnknownValueError:
    print("Could not understand audio.")
except sr.RequestError as e:
    print("Speech recognition service error:", e)

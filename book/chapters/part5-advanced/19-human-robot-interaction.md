# Chapter 19: Human-Robot Interaction

## Learning Objectives

1. **Implement** teleoperation interfaces for humanoid control
2. **Integrate** speech recognition and synthesis
3. **Deploy** gesture recognition for natural interaction
4. **Build** safety-aware HRI behaviors
5. **Evaluate** user experience and interaction quality

---

## 1. HRI Modalities

**Input Channels**:
- Speech: "Pick up the red cup"
- Gestures: Pointing, hand signals
- Teleoperation: VR controllers, joysticks
- GUI: Touchscreen interfaces

**Output Channels**:
- Speech: Text-to-speech responses
- Display: Status information
- Motion: Acknowledge with head nod

---

## 2. Speech Interface

### Install Dependencies
```bash
pip install speechrecognition pyttsx3
sudo apt install portaudio19-dev
```

### Speech-to-Text
```python
import speech_recognition as sr

def listen_for_command():
    """Capture speech and convert to text."""
    recognizer = sr.Recognizer()

    with sr.Microphone() as source:
        print("Listening...")
        audio = recognizer.listen(source, timeout=5)

    try:
        command = recognizer.recognize_google(audio)
        return command
    except sr.UnknownValueError:
        return None

# Usage
command = listen_for_command()
print(f"Command: {command}")
```

### Text-to-Speech
```python
import pyttsx3

def speak(text):
    """Convert text to speech."""
    engine = pyttsx3.init()
    engine.setProperty('rate', 150)  # Speed
    engine.say(text)
    engine.runAndWait()

speak("Task completed successfully")
```

---

## 3. Gesture Recognition

### MediaPipe Hand Tracking
```python
import cv2
import mediapipe as mp

class GestureRecognizer:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7
        )

    def detect_gesture(self, image):
        """Detect hand gesture from image."""
        results = self.hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        if not results.multi_hand_landmarks:
            return None

        # Get hand landmarks
        landmarks = results.multi_hand_landmarks[0]

        # Classify gesture (simplified)
        if self.is_pointing(landmarks):
            return "POINT"
        elif self.is_thumbs_up(landmarks):
            return "APPROVE"
        elif self.is_stop_sign(landmarks):
            return "STOP"

        return None

    def is_pointing(self, landmarks):
        # Index finger extended, others closed
        index_tip = landmarks.landmark[8]
        index_mcp = landmarks.landmark[5]
        return index_tip.y < index_mcp.y  # Simplified
```

---

## 4. Teleoperation

### VR Controller Integration
```python
import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class TeleoperationNode(Node):
    def __init__(self):
        super().__init__('teleoperation')

        # Subscribe to VR controller
        self.create_subscription(Twist, '/vr_controller/cmd_vel', self.vr_callback, 10)

        # Publish to robot
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)

    def vr_callback(self, msg):
        """Map VR controller input to robot joints."""
        # Map linear velocity to walking
        # Map angular velocity to turning

        joint_msg = JointState()
        # ... (mapping logic)
        self.joint_pub.publish(joint_msg)
```

---

## 5. Multimodal Integration

### Command Fusion
```python
class MultimodalInterface:
    def __init__(self):
        self.speech = SpeechRecognizer()
        self.gesture = GestureRecognizer()
        self.vla = OpenVLA()

    def process_command(self, image, audio):
        """Combine speech and gesture for robust commands."""

        # Speech: "Pick up the"
        speech_cmd = self.speech.recognize(audio)

        # Gesture: Pointing at object
        gesture = self.gesture.detect(image)

        # Combine
        if "pick up" in speech_cmd and gesture == "POINT":
            # Use VLA with pointing location
            target = self.get_pointed_object(image, gesture)
            action = self.vla.predict_action(
                image,
                f"Pick up the object at {target}"
            )
            return action

        return None
```

---

## 6. Safety in HRI

### Proxemics (Personal Space)
```python
def check_human_proximity(human_pos, robot_pos):
    """Slow down if human is too close."""
    distance = np.linalg.norm(human_pos - robot_pos)

    if distance < 0.5:  # < 0.5m: Stop
        return 0.0
    elif distance < 1.0:  # 0.5-1m: Slow
        return 0.3
    else:  # > 1m: Normal
        return 1.0

speed_factor = check_human_proximity(human, robot)
robot.set_velocity_scale(speed_factor)
```

### Emergency Stop
```python
class SafetyMonitor:
    def __init__(self):
        self.emergency_phrases = ["stop", "freeze", "emergency"]

    def check_speech_command(self, command):
        """Emergency stop on safety keywords."""
        if any(phrase in command.lower() for phrase in self.emergency_phrases):
            robot.emergency_stop()
            speak("Stopping immediately")
            return True
        return False
```

---

## 7. Hands-On Lab: Voice-Controlled Grasping (3 hours)

**Goal**: Control robot grasping with voice commands.

**Steps**:
1. Setup speech recognition
2. Integrate with VLA (Chapter 16)
3. Add confirmation responses (TTS)
4. Test commands: "Pick up the cup", "Put down the bottle"
5. Measure task success rate

**Validation**: >70% command recognition, >60% task success

---

## 8. End-of-Chapter Project

Build multimodal HRI system for household assistance.

**Requirements**:
- Speech interface (3+ commands)
- Gesture recognition (point, approve, stop)
- VLA execution
- Safety monitoring
- User study with 3 participants

**Deliverables**:
- HRI system code
- User study results (task completion time, errors)
- Demo video (3 multimodal interactions)

---

## Summary

Human-robot interaction combines speech, gestures, and visual interfaces for natural robot control. Safety-aware behaviors ensure comfortable human-robot collaboration.

**Next**: Chapter 20 covers safety standards and compliance for real-world deployment.

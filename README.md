# 🗑️ Collaborative Recycling Sorting Robot — 버려Dream

> **Doosan Robotics Bootcamp (2025) — Collaborative Robot 2nd Project**  
> A voice-controlled robotic system that automatically detects and sorts recyclable waste using AI vision and a collaborative robot arm.

---

## 📌 Project Overview

This project implements a smart recycling sorting robot that combines **voice recognition**, **AI-based object detection**, and **robotic manipulation** to automate the process of separating recyclable materials.

The robot listens for a wake word, understands natural language commands via GPT-4o, detects recyclables using a custom-trained YOLOv11 model through a RealSense depth camera, and sorts them into the correct bin using a Doosan M0609 collaborative robot arm.

---

## 🛠 Tech Stack

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-green)
![YOLO](https://img.shields.io/badge/YOLOv11-Ultralytics-purple)
![OpenAI](https://img.shields.io/badge/OpenAI-GPT--4o-orange)
![RealSense](https://img.shields.io/badge/Intel-RealSense-lightblue)

| Category | Technology |
|----------|-----------|
| Robot | Doosan M0609 Collaborative Robot |
| Gripper | OnRobot RG2 |
| Camera | Intel RealSense (Depth + RGB) |
| Object Detection | YOLOv11 (custom trained) |
| Voice Recognition | Google Speech-to-Text API |
| TTS | Google gTTS + Pygame |
| NLP / LLM | GPT-4o via LangChain |
| Wake Word | Custom wake word detection |
| Framework | ROS2 Humble |
| Language | Python 3.10 |

---

## 👥 Team & My Contributions

| | |
|--|--|
| **Team Name** | 버려Dream |
| **Project** | Doosan Robotics Bootcamp 2025 — Collaborative Robot 2nd Assignment |
| **Duration** | 3 days |

### ⭐ My Role

| Area | Contribution |
|------|-------------|
| 🎯 Object Detection | Dataset selection, custom data collection & YOLOv11 model training |
| 📷 Depth Processing | Real-time depth-based bin fullness detection & alert system |
| 🤖 Robot Control | Pick & place motion logic implementation |
| 🔲 Bounding Box | Gripper rotation logic based on bounding box ratio & exception handling |

---

## 🎯 Implementation Details

### 1. YOLOv11-based Object Detection ⭐

**Model Selection**

Detection accuracy was the top priority for this project. YOLOv11 was selected because:
- Supports real-time inference with a lightweight model architecture
- Well-suited for detecting diverse waste objects in an automated pipeline
- Best balance between speed and accuracy among evaluated models

**Dataset Selection & Training**

Unlike standard CNN classification datasets, object detection requires bounding box annotations for each object. Training data must also closely match the real deployment environment (country, region, lighting, specific objects) to perform reliably in practice.

Three datasets were trained and compared. The model with the lowest validation loss was selected as the final model.

| Category | Details |
|----------|---------|
| Target classes | Plastic, Can, Paper, Glass *(Glass excluded in live experiment for safety)* |
| Dataset source | [Roboflow — IT Project Recycle 4Section](https://universe.roboflow.com/kongju-university-se97i/it-project-recycle-4section/dataset/7/images?split=test) |
| Custom dataset | Self-collected photos of common Korean recyclables |

**Korea-Specific Dataset Problem & Solution** 💡

Korean plastic bottles are typically transparent and label-free. Under lab lighting or sunlight, they become nearly invisible to the model — causing frequent detection failures with the public Roboflow dataset.

To solve this, I **proposed and built a custom dataset** by personally photographing plastic bottles, cans, and paper commonly used in Korea, and retrained the model on this data.

| Dataset | Detection Accuracy |
|---------|-------------------|
| Roboflow public dataset | 79% |
| **Custom Korean dataset (self-collected)** | **98%** |

This 19%p improvement in accuracy was a direct result of aligning training data with the real deployment environment.

---

### 2. RealSense Camera — 3D Position Estimation & Bin Monitoring ⭐

**3D Position Estimation**
- Intel RealSense captures both RGB and depth frames simultaneously
- Detected object coordinates (from YOLO bounding box center) are projected into 3D space using depth values
- Camera coordinates are transformed to the robot base frame using a **hand-eye calibration matrix** (`T_gripper2camera.npy`)

**Real-time Bin Fullness Detection** 💡

In real-world recycling scenarios (homes, recycling stations), bins need to be emptied promptly when full. To address this, I implemented a **real-time depth-based bin monitoring system** that alerts the user when a bin is full.

To prevent the bin monitoring from blocking the robot control pipeline, both nodes run in parallel using **`MultiThreadExecutor()`**:
- Without parallel execution, the bin monitoring loop would block the main robot control node, causing delays in pick & place operations
- `MultiThreadExecutor()` allows both nodes to spin simultaneously, enabling real-time alerts during robot operation without any interference

---

### 3. Doosan M0609 — Stable Grip & Sorting Logic ⭐

**Adaptive Gripper Rotation**

Since objects can be placed in any orientation (unpredictable), a fixed gripper angle would result in unstable or failed grasps. To solve this:
- The robot moves to the **center coordinate of the YOLO bounding box**
- The **aspect ratio of the bounding box** is calculated
- The gripper **rotates to grasp along the shorter axis**, maximizing grip stability
- This improves adaptability for randomly oriented or deformed objects

**Motion Compensation for Difficult Objects**

For objects that are low-profile or highly reflective (e.g., crushed cans), depth estimation can be inaccurate. A **relative motion compensation** is applied to adjust the pick position after initial movement, ensuring reliable grasping.

**Deep Placement Motion**

After sorting, the robot lowers the object fully into the bin using a height-adjusted placement motion — preventing objects from bouncing out of the bin.

**Exception Handling**

If the number of objects detected by YOLO differs from the number of objects in the voice command, the system handles the mismatch gracefully and logs a warning.

---

### 4. Voice Interface — STT + LLM + TTS

**Speech-to-Text (STT)**
- Activated by a **custom wake word**
- **Google Speech-to-Text API** converts user speech to text in real time

**Natural Language Understanding**
- Converted text is passed to **GPT-4o via LangChain** with a structured prompt
- GPT-4o extracts only the relevant information (target objects) based on defined rules
- Supports natural Korean commands:
  - *"플라스틱 버려줘"* → `plastic`
  - *"쓰레기 다 버려줘"* → `can, paper, plastic`

**Text-to-Speech (TTS)**
- **Google gTTS** synthesizes voice responses
- **Pygame** plays the generated mp3 audio for user feedback

---

## 🏗️ System Architecture

```
Voice Input (Wake Word)
        │
        ▼
Google STT → GPT-4o (LangChain) → Target Object Extracted
                                            │
                                            ▼
                                 RealSense RGB + Depth
                                            │
                                   YOLOv11 Detection
                                            │
                              3D Position Estimation
                              (Hand-Eye Calibration)
                                            │
                                 Doosan M0609 Robot
                          (Adaptive Grip + Deep Placement)
                                            │
                         ┌──────────────────┼──────────────────┐
                         ▼                  ▼                  ▼
                      Can Bin          Paper Bin          Plastic Bin

        [Parallel Thread — MultiThreadExecutor]
        Bin Fullness Monitor (RealSense Depth) → TTS Alert
```

---

## 📁 Package Structure

```
dream/
├── object_detection/
│   ├── detection.py       # Depth-based 3D position estimation
│   ├── yolo.py            # YOLOv11 inference & multi-frame aggregation
│   └── realsense.py       # RealSense camera interface
├── robot_control/
│   ├── robot_control.py   # Main pick & place controller (ROS2 Node)
│   └── onrobot.py         # OnRobot RG2 gripper control
├── voice_processing/
│   ├── get_keyword.py     # Voice pipeline: wake word → STT → GPT-4o
│   ├── stt.py             # Google STT wrapper
│   ├── wakeup_word.py     # Wake word detection
│   └── MicController.py   # Microphone stream management
├── resource/
│   ├── garbage_sorting.json      # YOLO class label mapping
│   └── T_gripper2camera.npy      # Hand-eye calibration matrix
├── package.xml
└── setup.py
```

---

## 🚀 Getting Started

### Prerequisites
- ROS2 Humble
- Doosan Robot ROS2 package (`DSR_ROBOT2`)
- Intel RealSense SDK
- Python 3.10+

### Installation

```bash
# Clone the repository
git clone https://github.com/seojinhg/Recycling-sorting-robot.git
cd Recycling-sorting-robot

# Install Python dependencies
pip install ultralytics openai langchain pyaudio python-dotenv scipy pygame gtts

# Set up environment variables
cp ros2_ws/src/dream/resource/.env.example ros2_ws/src/dream/resource/.env
# Add your OpenAI API key and Google API key to .env

# Build the ROS2 workspace
cd ros2_ws
colcon build
source install/setup.bash
```

### Run

```bash
# Launch the robot system
ros2 launch dream dream.launch.py

# In a separate terminal, start the robot controller
ros2 run dream robot_control
```

---

## 📊 Results

| Item | Result |
|------|--------|
| Detection Accuracy (Roboflow dataset) | 79% |
| Detection Accuracy (Custom Korean dataset) | **98%** |
| Recyclable categories (live) | Can, Paper, Plastic |
| Voice → Pick & Place full pipeline | ✅ Fully operational |
| Bin fullness real-time detection | ✅ Depth-based alert working |
| Adaptive gripper rotation | ✅ Bounding box ratio-based |

---

## 🎬 Demo

> 📹 Demo video coming soon

---

## 🔧 Limitations & Future Work

Due to the **3-day implementation constraint**, the following safety feature was planned but not completed:

**Force Detection / Safety Stop**

When a person approaches the robot during operation — for example, to empty a full bin or sort alongside the robot — there is currently no mechanism to detect unexpected contact or halt motion safely.

A `force_detection` module was planned to stop the robot immediately upon detecting external force, which is critical for safe human-robot collaboration in shared spaces. This remains a priority for future development.

---

## 📝 Notes

- YOLO model weights (`.pt`) and large binary files are excluded from this repository due to size constraints.
- Environment variables (API keys) are managed via `.env` — never committed to the repository.
- The robot is configured for **Doosan M0609** — modify joint positions in `robot_control.py` for other models.
- Glass was excluded from the live experiment for safety reasons.

# 🦆 Duckiebot Autonomous Driving Simulation in Isaac Sim

> Intelligent Autonomous Robot (Duckiebot) Project in NVIDIA Isaac Sim
> 
> 
> An integrated autonomous driving system based on **ROS 2**, utilizing control theory, sensor processing, and computer vision (OpenCV) to implement color-based recognition and autonomous navigation.
> 

---

## 📝 Project Overview

This project aims to precisely implement the Duckiebot robot model within the **NVIDIA Isaac Sim** virtual environment and independently develop a full **Perception - Decision - Control** pipeline using **ROS 2 Humble**.

By leveraging the modular architecture of ROS 2, we have integrated the Duckiebot's control, recognition, and autonomous driving functions. The system includes an LED control service, low-level/high-level motor control for precise movement, and bandwidth-efficient image compression with a real-time viewer.

The core feature, **Vision-based Autonomous Driving**, utilizes HSV filtering to analyze the centroid of a "Red Cube." It performs tracking using P-control and features an **Active Search** algorithm that rotates the robot in place to reacquire the target if it disappears from the field of view, ensuring driving stability.

---

## 2. 📂 Directory Structure

```jsx
~/duck_ws/src
├── duckie_chase       # [Decision] Autonomous driving judgment & tracking algorithms
├── duckie_control     # [Control] Motor control & Inverse Kinematics driver
├── duckie_vision      # [Perception] Image compression & Custom viewer
├── duckie_led         # [Utility] LED control service
└── duckie_low_level   # [Debug] Direct hardware control testing
```

---

## 3. 💻 Environment

This project was developed and tested in the following environment. Using the same versions is recommended for compatibility.

| **Component** | **Version** | **Note** |
| --- | --- | --- |
| **OS** | Ubuntu 22.04 LTS |  |
| **Middleware** | ROS 2 Humble |  |
| **Simulator** | NVIDIA Isaac Sim | **4.5.0** |
| **Language** | Python | 3.10.12 |
| **Library** | NumPy | **< 2.0** (1.26.x recommended) |

> ⚠️ Warning: The cv_bridge package in ROS 2 Humble is built against NumPy 1.x. If NumPy 2.x is installed, compatibility errors will occur. You must downgrade NumPy as shown below.
> 

---

## 4. 🚀 Installation & Usage

### Step 1. Install Essential Libraries (Important)

Adjust the NumPy version for ROS 2 compatibility.

```jsx
# Uninstall latest NumPy and install version < 2.0
pip3 uninstall numpy
pip3 install "numpy<2.0"
```

### Step 2. Build Workspace

Download and build the project.

```jsx
# Move to workspace (Modify path according to your environment)
cd ~/duck_ws

# Build packages
colcon build --symlink-install

# Source the environment
source install/setup.bash
```

### Step 3. Launch Isaac Sim

1. Launch **Isaac Sim 4.5.0**.
2. Load the configured Duckiebot stage (USD file) and press the **Play** button.
    - *Ensure the ROS 2 Bridge is active.*

### Step 4. Run ROS 2 Nodes

Open a new terminal for each command below and run them in order.

(Note: Always run source install/setup.bash before executing commands).

### 1. duckie_led (Change LED Color)

> A package to change the color of the LEDs attached to the Duckiebot in Isaac Sim via ROS 2 Service.
> 

```jsx
# Terminal 1
ros2 run duckie_led led_server.py

# Terminal 2
ros2 service call /duckie_led_control duckie_led/srv/SetColor "color: 'blue'"
```

### 2. duckie_low_level (Low-level Control)

> Receives movement commands (Twist), converts them into wheel rotation speeds, and sends them to the simulator.
> 

```jsx
# Terminal 1
ros2 run duckie_low_level raw_wheel_operator.py

# Terminal 2
ros2 topic echo /duckie/wheel_left_cmd

# Terminal 3
ros2 topic echo /duckie/wheel_right_cmd
```

### 3. duckie_control (High-level Control - Teleop)

> Uses the motor driver to control the robot via keyboard commands.
> 

```jsx
# Terminal 1
ros2 run duckie_control motor_driver.py 

# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/duckie/cmd_vel
```

### 4. duckie_vision (Camera Signal Acquisition & Publish)

> Publishes the image_raw topic and enables the use of compressed images.
> 

```jsx
# Terminal 1
ros2 run duckie_vision image_compressor

# Terminal 2
# (Allows viewing of compressed images. Reflects Isaac Sim changes in real-time)
ros2 run duckie_vision simple_viewer

# Terminal 3 - View image_raw via rqt
ros2 run rqt_image_view rqt_image_view
```

### 5. duckie_chase (Image Processing & Autonomous Control)

> Analyzes video to detect a cube and issues driving commands.
> 
> - Displays the **Centroid** and **Bounding Box**.
> - **Logic:** Left turn if the cube is left of center, Right turn if right of center, Go straight if centered.
> - **Active Search:** If the cube is not visible, the robot rotates in place until the target is identified.

```jsx
# Terminal 1
ros2 run duckie_vision image_compressor

# Terminal 2
python3 ~/duck_ws/src/duckie_vision/duckie_vision/simple_viewer.py 

# Terminal 3 (Window opens showing the detection)
ros2 run duckie_chase detection_node
```


# 🦆 Duckiebot Autonomous Driving Simulation in Isaac Sim

> NVIDIA Isaac Sim 환경에서 구현한 지능형 자율주행 로봇(Duckiebot) 프로젝트 > ROS 2 기반의 제어, 센서 처리, 컴퓨터 비전(OpenCV)을 통합하여 색상 인식 기반의 자율주행 시스템을 구축했습니다.
> 

---

## 📝 프로젝트 개요 (Project Overview)

본 프로젝트는 **NVIDIA Isaac Sim** 가상 환경에서 Duckiebot 로봇 모델을 정밀하게 구현하고, ROS 2 미들웨어를 활용하여 **인지(Perception) - 판단(Decision) - 제어(Control)**의 전체 파이프라인을 독자적으로 개발하는 것을 목표로 합니다.


# 🦆 Duckiebot Autonomous Driving in Isaac Sim

## 1. 📝 프로젝트 개요 (Project Overview)

본 프로젝트는 **NVIDIA Isaac Sim** 가상 환경에서 Duckiebot 로봇 모델을 구현하고,**ROS 2 Humble**을 기반으로 **지능형 자율주행 시스템**을 구축하는 것을 목표로 합니다.

본 프로젝트는 ROS 2의 모듈화된 아키텍처를 활용하여 Duckiebot의 제어, 인식, 자율주행 기능을 통합적으로 구현했습니다. LED 제어 서비스와 저수준/고수준 모터 제어를 통해 로봇의 상태 표시 및 정밀 구동이 가능하며, 대역폭 효율화를 위한 이미지 압축 전송 및 실시간 뷰어를 구축했습니다. 핵심 기능인 **영상 기반 자율주행**은 HSV 필터링으로 '빨간색 큐브'의 중심 좌표(Centroid)를 분석하여 P-제어 기반의 추적 주행을 수행하며, 목표물이 시야에서 사라질 경우 제자리에서 회전하며 대상을 재탐색하는 능동형 탐색(Active Search) 알고리즘을 탑재하여 주행 안정성을 확보했습니다.

---

### 2. 📂 디렉토리 구조 (Directory Structure)

Bash

```bash
~/duck_ws/src
├── duckie_chase       # [Decision] 자율주행 판단 및 추적 알고리즘
├── duckie_control     # [Control] 모터 제어 및 역운동학 드라이버
├── duckie_vision      # [Perception] 이미지 압축 및 커스텀 뷰어
├── duckie_led         # [Utility] LED 제어 서비스
└── duckie_low_level   # [Debug] 하드웨어 직접 제어 테스트
```

## 3. 💻 개발 환경 (Environment)

이 프로젝트는 다음 환경에서 개발 및 테스트되었습니다. 호환성을 위해 동일한 버전을 권장합니다.

| **Component** | **Version** | **Note** |
| --- | --- | --- |
| **OS** | Ubuntu 22.04 LTS |  |
| **Middleware** | ROS 2 Humble |  |
| **Simulator** | NVIDIA Isaac Sim | **4.5.0** |
| **Language** | Python | 3.10.12 |
| **Library** | NumPy | **< 2.0** (1.26.x 권장) |

> ⚠️ 주의사항: ROS 2 Humble의 cv_bridge는 NumPy 1.x 버전을 기준으로 빌드되어 있습니다. NumPy 2.x가 설치된 환경에서는 호환성 오류가 발생하므로 반드시 다운그레이드가 필요합니다. (아래 실행 방법 참조)
> 

---

## 4. 🚀 실행 방법 (Installation & Usage)

### Step 1. 필수 라이브러리 설정 (중요)

ROS 2와의 호환성을 위해 NumPy 버전을 조정해야 합니다.

```bash
# 최신 NumPy 삭제 및 2.0 미만 버전 설치
pip3 uninstall numpy
pip3 install "numpy<2.0"
```

### Step 2. 워크스페이스 빌드

프로젝트를 다운로드하고 빌드합니다.

```bash
# 워크스페이스로 이동 (경로는 사용자 환경에 맞게 수정)
cd ~/duck_ws

# 패키지 빌드
colcon build --symlink-install

# 환경 설정 적용
source install/setup.bash
```

### Step 3. Isaac Sim 실행

1. Isaac Sim 4.5.0을 실행합니다.
2. 구성해 둔 Duckiebot 스테이지(USD 파일)를 로드하고 **Play** 버튼을 누릅니다.
    - *ROS 2 Bridge가 활성화되어 있어야 합니다.*

### Step 4. ROS 2 노드 실행

각각 새로운 터미널을 열어 아래 명령어들을 순서대로 실행합니다. (실행 전 `source install/setup.bash` 필수)

**1. duckie_led (LED 색 변경)**

> Isaac Sim의 Duckiebot에 부착된 LED 색을 ROS2 Service를 통해 변경하는 패키지입니다.
> 

```bash
#Terminal1
ros2 run duckie_led led_server.py

#Terminal2
ros2 service call /duckie_led_control duckie_led/srv/SetColor "color: 'blue'"
```

**2. duckie_low_level (duckiebot 저수준 제어)**

> 이동 명령(Twist)을 받아 바퀴의 회전 속도로 변환하여 시뮬레이터로 보냅니다.
> 

```bash
#Terminal1
ros2 run duckie_low_level raw_wheel_operator.py

#Terminal2
ros2 topic echo /duckie/wheel_left_cmd

#Terminal3
ros2 topic echo /duckie/wheel_right_cmd
```

**3. duckie_control (duckiebot 고수준 제어-Teleop Twist Keyboard)**

> 영상을 분석해 큐브를 찾고 주행 명령을 내립니다. (빨간 큐브 추적 시작)
> 

```bash

#터미널1
ros2 run duckie_control motor_driver.py 

#터미널 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/duckie/cmd_vel

```

**4. duckie_vision (카메라 신호 획득 및 ROS2 publish)**

> image_raw 토픽을 publish하고 compressed(압축 이미지) 사용 가능합니다.
> 

```bash

#Terminal 1
ros2 run duckie_vision image_compressor

#Terminal 2
ros2 run duckie_vision simple_viewer
(압축 이미지 볼 수 있음) 아이작 심 환경에서 변경사항있어도 계속 받아오기 때문에 실시간 반영됨

#Terminal 3 - rqt viewer에서 image_raw로 볼 수 있음
ros2 run rqt_image_view rqt_image_view

```

**5. duckie_chase (영상처리 후, duckiebot 행동 제어)**

> 영상을 분석해 큐브를 찾고 주행 명령을 내립니다. Centroid, Bounding box 볼 수 있으며 화면 중앙보다 왼쪽에 큐브가 있으면 좌회전, 오른쪽에 있으면 우회전, 정면에 존재하면 직진합니다. 또한, 화면에 큐브가 없을 경우, 제자리 회전으로 큐브를 식별할 때까지 회전합니다.
> 

```bash
#Terminal 1
ros2 run duckie_vision image_compressor

#Terminal 2
python3 ~/duck_ws/src/duckie_vision/duckie_vision/simple_viewer.py 

#Terminal 3 (창 뜨고 큐브 보임)
ros2 run duckie_chase detection_node

```

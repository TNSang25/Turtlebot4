# TurtleBot 4 — Phân vùng bản đồ và điều hướng bằng giọng nói

ROS 2 Humble / Gazebo Fortress. OpenCV phân vùng bản đồ đã lưu; Whisper
nhận lệnh tiếng Việt offline; Nav2 đưa robot đến khu vực được yêu cầu.

## 1. Các package

| Package | Chức năng / node chính |
| --- | --- |
| `robot_description` | URDF, mesh và cảm biến TurtleBot 4. |
| `robot_sim` | Mở Gazebo, spawn robot và bridge `/cmd_vel`, `/odom`, `/scan`, `/clock`. |
| `robot_mapping` | SLAM Toolbox và lưu bản đồ. |
| `robot_navigation` | Nạp map, định vị AMCL và chạy Nav2. Bản đồ lưu trong `maps/`. |
| `robot_semantic` | `segment_map` phân vùng; `semantic_map_node` hiển thị vùng và điểm đích trên RViz. |
| `robot_voice` | `speech_recognition_node` đọc micro; `voice_navigation_node` phân tích lệnh và gửi action `/navigate_to_pose`. |
| `robot_node` | Teleop, ghi goal, điều hướng waypoint và điều khiển hành vi cử chỉ. |
| `robot_gesture` / `robot_perception` | Nhận diện cử chỉ MediaPipe / vật thể YOLO từ các bài trước. |

Luồng chính: **micro → Whisper → `/voice/text` → phân tích vùng → Nav2**.

## 2. Cài đặt

Yêu cầu Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress và các package trong
`~/workspaces/turtlebot4_ws/src/`.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-venv python3-pip \
  python3-opencv python3-numpy python3-yaml libportaudio2 \
  ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup

cd ~/workspaces/turtlebot4_ws
python3 -m venv --system-site-packages .venv-voice
.venv-voice/bin/python -m pip install -r src/robot_voice/requirements.txt
python3 src/robot_voice/scripts/download_model.py

source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install
```

Model Whisper small tải khoảng 464 MB, được kiểm tra checksum và chỉ tải
một lần trước khi build. Khi nhận dạng, âm thanh được xử lý tại máy.

## 3. Bản đồ và khu vực

Đã có ba vùng trên `robot_navigation/maps/my_map.yaml`, tương ứng world
`demo`. Xem ảnh [phân vùng A/B/C](robot_semantic/maps/regions_preview.png).

| ID | Tên / câu lệnh | Điểm đích `(x, y)` trong frame `map` |
| --- | --- | --- |
| A | Phòng học / khu vực A | `(-1.475, 2.485)` |
| B | Hành lang / khu vực B | `(-0.475, -0.015)` |
| C | Phòng thí nghiệm / khu vực C | `(2.525, 2.485)` |

Tên chức năng do người dùng gán trong `robot_semantic/config/seeds.yaml`.
Thuật toán threshold theo map YAML, morphology rồi lan truyền nhãn trên ô
trống theo khoảng cách đường đi; không lan xuyên tường. Điểm đích được kiểm
tra khoảng trống. Đây là phân vùng chức năng, không tự suy diễn công năng phòng.

Để chỉnh tên, bí danh hoặc điểm hạt giống, sửa `seeds.yaml`, rồi tạo lại:

```bash
source install/setup.bash
ros2 run robot_semantic segment_map \
  --map src/robot_navigation/maps/my_map.yaml \
  --seeds src/robot_semantic/config/seeds.yaml \
  --output src/robot_semantic/config/regions.yaml
```

Với map khác, có thể bỏ `--seeds` và thêm `--count 3` để chọn hạt giống tự
động. Kiểm tra ảnh kết quả, chỉnh tên/bí danh trong YAML xuất ra và truyền
`regions_file:=...` khi launch. Map nạp phải khớp bản đã phân vùng.

## 4. Thứ tự chạy

Chuẩn bị **mỗi terminal**:

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
```

**Terminal 1 — Simulation:** dùng đúng world đã lập bản đồ.

```bash
ros2 launch robot_sim sim.launch.py world:=demo enable_camera:=false
```

**Terminal 2 — Localization:**

```bash
ros2 launch robot_navigation localization.launch.py \
  map:=$PWD/src/robot_navigation/maps/my_map.yaml
```

Trong RViz, dùng **2D Pose Estimate** đặt đúng vị trí/hướng robot và kiểm tra
`/scan` khớp tường. Display **Semantic Regions** sẽ hiện khi chạy terminal 4.

**Terminal 3 — Nav2:**

```bash
ros2 launch robot_navigation navigation.launch.py
```

**Terminal 4 — Phân vùng và nhận lệnh từ micro:**

```bash
ros2 launch robot_voice voice_navigation.launch.py \
  python_executable:=$PWD/.venv-voice/bin/python \
  model_path:=$PWD/src/robot_voice/models/faster-whisper-small
```

Chờ `Microphone ready` và trạng thái điều hướng `ready`. Nói từng câu, ngừng
khoảng 0,8 giây và chờ dòng `HEARD`:

- **“Đi đến khu vực A.”**
- **“Di chuyển từ khu vực A đến khu vực B.”** Nếu chưa ở A, robot đến A trước.
- **“Đi đến phòng thí nghiệm.”**
- **“Dừng lại.”** Hủy hành trình đang chạy.

Khu vực không tồn tại sẽ bị từ chối. Khi robot đang chạy, lệnh mới không thay
thế hành trình; nói dừng trước khi đổi lệnh. Không chạy teleop, controller cử
chỉ hoặc node waypoint đồng thời. Whisper chạy CPU nên cần chờ nhận dạng xong
trước câu tiếp theo.

Nếu chọn micro khác, thêm `device:=<index hoặc tên>`. Xem danh sách:

```bash
.venv-voice/bin/python -m sounddevice
```

Theo dõi kết quả:

```bash
ros2 topic echo /voice/text
ros2 topic echo /voice/navigation_status
ros2 topic echo /voice/speech_status
```

Thêm `speech:=false` để chỉ kiểm tra parser/Nav2 bằng topic văn bản; không
thay thế phần demo micro. `audio_file:=/duong/dan/lenh.wav` dùng để phát lại
một câu đã thu. Nhãn vùng: `/semantic_map/markers`; ảnh: `/semantic_map/image`.

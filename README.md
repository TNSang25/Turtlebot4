# TurtleBot 4 — Điều khiển bằng cử chỉ tay

ROS 2 Humble, Gazebo Fortress, webcam laptop và MediaPipe. Robot nhận cử chỉ
chọn hành vi, chờ xác nhận rồi thực hiện. Repo có thêm SLAM, Nav2 và YOLO.

## 1. Các package

| Package | Chức năng / node chính |
| --- | --- |
| `robot_description` | URDF, mesh chính thức, cảm biến; `robot_state_publisher`. |
| `robot_sim` | World, spawn robot, bridge ROS–Gazebo; topic `/cmd_vel`, `/odom`, `/scan`, `/camera/image`. |
| `robot_mapping` | Xây dựng và lưu bản đồ bằng `slam_toolbox`. |
| `robot_navigation` | Bản đồ trong `maps/`, định vị AMCL và điều hướng Nav2. |
| `robot_node` | `robot_behavior_node` thực hiện hành vi; thêm teleop, ghi goal và điều hướng qua waypoint. |
| `robot_perception` | `yolo_detector` nhận diện vật thể từ camera robot. |
| `robot_gesture` | `gesture_recognition_node` đọc webcam, nhận diện và gửi `/detected_gesture`. |

Luồng điều khiển: **webcam → gesture_recognition_node → /detected_gesture
→ robot_behavior_node → /cmd_vel → robot trong Gazebo**.

## 2. Clone và cài đặt

Yêu cầu **Ubuntu 22.04, ROS 2 Humble và Gazebo Fortress** đã cài đặt.

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-venv python3-pip \
  python3-yaml python3-opencv libportaudio2 \
  ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-rviz2 ros-humble-cv-bridge \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

Clone nhánh `assignment_6` vào `src/` của **workspace mới**. Nếu đã có code,
bỏ qua bước clone.

```bash
mkdir -p ~/workspaces/turtlebot4_ws
cd ~/workspaces/turtlebot4_ws
git clone --branch assignment_6 https://github.com/TNSang25/Turtlebot4.git src
```

Cài MediaPipe trong môi trường riêng, tải model và build:

```bash
cd ~/workspaces/turtlebot4_ws
python3 -m venv --system-site-packages .venv-gesture
.venv-gesture/bin/python -m pip install -r src/robot_gesture/requirements.txt
python3 src/robot_gesture/scripts/download_model.py

source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install
```

## 3. Các bước chạy

**Chuẩn bị mỗi terminal:**

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
```

**Bước 1 — Terminal 1: mở mô phỏng.** World `demo` có vùng trống cho các hành vi.
Bài này dùng webcam laptop nên có thể tắt camera trên robot.

```bash
ros2 launch robot_sim sim.launch.py world:=demo enable_camera:=false
```

**Bước 2 — Terminal 2: mở webcam và node điều khiển.**

```bash
ros2 launch robot_gesture gesture_control.launch.py \
  python_executable:=$PWD/.venv-gesture/bin/python \
  model_path:=$PWD/src/robot_gesture/models/gesture_recognizer.task \
  control_robot:=true
```

Đổi `camera_index:=1` nếu cần chọn webcam khác. `control_robot:=false` chỉ
thử nhận diện; khi đó dòng `controller not connected` là bình thường.

**Bước 3 — Điều khiển bằng tay.** Đưa một tay vào ảnh, giữ cử chỉ chọn khoảng
0,5 giây, chờ `Selected`, rồi giơ ngón cái để xác nhận.

| Cử chỉ | Hành vi |
| --- | --- |
| Vẫy bàn tay xòe qua lại vài nhịp | Chọn xoay trái–phải để chào. |
| Duỗi ngón trỏ | Chọn tiến–lùi 4 lần. |
| Duỗi ngón trỏ và ngón giữa thành chữ V | Chọn đi một vòng tròn. |
| Giơ ngón cái | Xác nhận và thực hiện lựa chọn. |
| Xòe bàn tay đứng yên | Dừng ngay. |
| Nắm tay, gập cả ngón cái | Hủy hành vi, trở về chờ. |

Giữ tay với cử chỉ hợp lệ trong ảnh khi robot chạy. Robot tự dừng khi mất tín
hiệu, thiếu dữ liệu cảm biến hoặc gặp vật cản gần. Không chạy teleop hoặc Nav2
cùng lúc với điều khiển cử chỉ. Đóng cửa sổ webcam hoặc `Ctrl+C` để kết thúc.

Terminal phụ kiểm tra:

```bash
ros2 topic echo /detected_gesture
ros2 topic echo /robot_behavior/state
ros2 topic echo /cmd_vel
```

Chi tiết nhận diện và thông số: [robot_gesture/README.md](robot_gesture/README.md).

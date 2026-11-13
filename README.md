# TurtleBot 4 — Điều khiển bằng biểu cảm khuôn mặt

Ubuntu 22.04, ROS 2 Humble và Gazebo Fortress. FER nhận diện biểu cảm từ
webcam laptop; robot thực hiện quỹ đạo tương ứng trong world `arena`.

**Pipeline:** webcam → `emotion_recognition_node` → `/detected_emotion`
→ `emotion_behavior_node` → `/cmd_vel` → `simulation_bridge` → Gazebo.

## 1. Package và node chính

| Package | Node / chức năng |
| --- | --- |
| `robot_emotion` | `emotion_recognition_node`: OpenCV đọc webcam, FER phân loại và hiển thị biểu cảm, độ tin cậy, trạng thái robot. |
| `robot_node` | `emotion_behavior_node`: nhận biểu cảm, kiểm tra LiDAR/odometry và gửi vận tốc ở 20 Hz. |
| `robot_sim` | Mở world, spawn robot và chạy `simulation_bridge` từ `ros_gz_bridge`. |
| `robot_description` | URDF, mesh và cảm biến TurtleBot 4. |

Các package `robot_mapping`, `robot_navigation`, `robot_semantic`,
`robot_voice`, `robot_gesture`, `robot_perception` cung cấp SLAM, Nav2, phân
vùng bản đồ, giọng nói, cử chỉ tay và YOLO. Hướng dẫn bài giọng nói nằm ở
[nhánh assignment_7](https://github.com/TNSang25/Turtlebot4/tree/assignment_7).

## 2. Topic chính

| Topic | Kiểu message | Nội dung |
| --- | --- | --- |
| `/detected_emotion` | `std_msgs/msg/String` | Nhãn biểu cảm chữ thường, ví dụ `happy`, `neutral`. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Vận tốc thẳng và góc cho robot. |
| `/odom` | `nav_msgs/msg/Odometry` | Theo dõi quãng đường và góc quay. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Kiểm tra vật cản. |
| `/emotion_behavior/state` | `std_msgs/msg/String` | JSON trạng thái `running`, `completed`, `stopped` và lý do dừng. |
| `/emotion/image` | `sensor_msgs/msg/Image` | Ảnh webcam đã chú thích. |
| `/emotion/recognition` | `std_msgs/msg/String` | JSON nhãn, độ tin cậy, số mặt và thời gian xử lý. |

## 3. Clone và cài đặt

Máy đã cài ROS 2 Humble. Clone các package trực tiếp vào `src/`:

```bash
mkdir -p ~/workspaces/turtlebot4_ws
cd ~/workspaces/turtlebot4_ws
git clone -b assigment_8 https://github.com/TNSang25/Turtlebot4.git src
```

Cài phụ thuộc và build các package cần cho bài biểu cảm:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-pip python3-venv \
  python3-yaml ros-humble-ros-gz ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-nav2-msgs ros-humble-rviz2

cd ~/workspaces/turtlebot4_ws
python3 src/robot_emotion/scripts/install_dependencies.py
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-up-to robot_sim robot_emotion
```

Script tạo `.venv-emotion`, cài FER 25.10.3 và TensorFlow CPU. Model TFLite
đi kèm FER, không cần tải riêng. Backend dùng OpenCV Haar, không cài các phụ
thuộc MTCNN/GPU hoặc xuất video không sử dụng.

## 4. Các bước chạy

**Trong mỗi terminal**, chuẩn bị môi trường:

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
export PYTHONNOUSERSITE=1
```

**Terminal 1 — Simulation:**

```bash
ros2 launch robot_sim sim.launch.py world:=arena enable_camera:=false
```

**Terminal 2 — Webcam và điều khiển:**

```bash
ros2 launch robot_emotion emotion_control.launch.py \
  python_executable:=$PWD/.venv-emotion/bin/python camera_index:=0
```

Giữ một khuôn mặt nhìn thẳng webcam, đủ sáng. Xem `Emotion`, `Confidence`,
`Robot` và `Running` trên cửa sổ. Nhãn chuyển động cần ổn định qua vài khung
hình mới kích hoạt. Không chạy teleop, Nav2 hoặc controller khác đồng thời.

| Biểu cảm | Hành vi mặc định |
| --- | --- |
| Happy | Cung trái 90°, bán kính 0,60 m. |
| Sad | Cung phải 90°, bán kính 0,60 m. |
| Neutral | Dừng và sẵn sàng nhận hành vi mới. |
| Angry | Lùi 0,25 m. |
| Surprise | Xoay tại chỗ một vòng theo odometry. |
| Fear | Tiến 0,25 m. |

Mỗi hành vi chạy một lần; về **Neutral** rồi thể hiện lại để lặp cùng hành
vi. Biểu cảm khác đã ổn định có thể thay hành vi đang chạy. Mất mặt, nhiều
mặt, mất tín hiệu, cảm biến cũ hoặc vật cản gần đều gây dừng; về Neutral để
chạy lại sau khi bị ngắt. Đóng cửa sổ hoặc Ctrl+C để thoát và dừng robot.

**Chỉ thử nhận diện**, không cần simulation:

```bash
ros2 launch robot_emotion emotion_control.launch.py control_robot:=false \
  python_executable:=$PWD/.venv-emotion/bin/python
```

**Theo dõi** ở terminal đã source cùng môi trường:

```bash
ros2 topic echo /detected_emotion
ros2 topic echo /emotion_behavior/state
```

Không mở được webcam: đóng ứng dụng đang dùng camera hoặc đổi `camera_index`.
Cấu hình ngưỡng, tốc độ và quỹ đạo tại
[emotion_control.yaml](robot_emotion/config/emotion_control.yaml).
Lớp `disgust` không có hành vi; kết quả không chắc chắn kéo dài cũng dừng robot.

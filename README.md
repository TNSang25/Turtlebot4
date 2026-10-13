# TurtleBot4 — ROS 2 Humble / Gazebo Fortress
## 1. Cấu trúc chương trình

```text
Turtlebot4/
├── README.md
├── robot_description/
│   ├── urdf/          # Xacro robot, cảm biến và plugin Gazebo
│   ├── meshes/        # Mesh TurtleBot4 và iRobot Create 3 chính thức
│   ├── config/        # simulation.yaml
│   ├── launch/        # display.launch.py
│   └── rviz/
├── robot_sim/
│   ├── worlds/        # World demo và empty
│   ├── maps/          # Map mẫu của world demo
│   ├── config/        # bridge_config.yaml, demo_layout.yaml
│   ├── launch/        # sim, gazebo, spawn, bridge
│   └── scripts/
├── robot_mapping/
│   ├── config/        # slam.yaml
│   ├── launch/        # mapping.launch.py, save_map.launch.py
│   └── rviz/
├── robot_navigation/
│   ├── maps/          # Bản đồ SLAM đã lưu: YAML và PGM
│   ├── config/        # nav2.yaml
│   ├── launch/        # localization.launch.py, navigation.launch.py
│   └── rviz/
└── robot_node/
    ├── robot_node/    # Node teleop và xử lý vận tốc
    ├── resource/
    └── test/
```

| Package | Chức năng |
| --- | --- |
| `robot_description` | Mô tả robot, khớp, frame, cảm biến và cấu hình mô phỏng. |
| `robot_sim` | Mở Gazebo, spawn robot và bridge các topic ROS–Gazebo. |
| `robot_mapping` | Chạy SLAM Toolbox và lưu bản đồ. |
| `robot_navigation` | Chạy localization và navigation bằng hai launch riêng. |
| `robot_node` | Điều khiển robot bằng bàn phím, gửi lệnh tới `/cmd_vel`. |

## 2. Clone và cài đặt

Môi trường: **Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress**.
Các lệnh dưới đây giả định ROS 2 Humble đã được cài đặt.

Cài công cụ build và dependency:

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-yaml \
  ros-humble-ros-gz ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard
```

```bash
mkdir -p ~/workspaces/turtlebot4_ws
cd ~/workspaces/turtlebot4_ws
git clone https://github.com/TNSang25/Turtlebot4.git src
```

## 3. Các bước chạy

### Chuẩn bị terminal

Mở 4 terminal. Trong **mỗi terminal**, chạy:

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
```

Giữ terminal ở thư mục `turtlebot4_ws` để các đường dẫn `$PWD/src/...` đúng.

### Mở simulation — terminal 1

```bash
ros2 launch robot_sim sim.launch.py enable_camera:=true
```

### Bước 1 — Mapping

Terminal 2 mở SLAM Toolbox và RViz:

```bash
ros2 launch robot_mapping mapping.launch.py
```

Terminal 3 mở teleop:

```bash
ros2 run robot_node teleop_keyboard
```

Focus terminal teleop, dùng các phím chữ thường (tắt gõ tiếng việt):

| Phím | Chức năng |
| --- | --- |
| `w` / `s` | Tiến / lùi |
| `a` / `d` | Quay trái / phải |
| `Space` | Dừng |
| `Ctrl+C` | Dừng và thoát |

Terminal 4 lưu map **khi mapping vẫn đang chạy**:

```bash
ros2 launch robot_mapping save_map.launch.py \
  map_name:=$PWD/src/robot_navigation/maps/my_map
```

Sau khi lưu xong, `Ctrl+C` tắt **teleop và mapping**, giữ simulation.

### Bước 2 — Localization

Terminal 2:

```bash
ros2 launch robot_navigation localization.launch.py \
  map:=$PWD/src/robot_navigation/maps/my_map.yaml
```

Launch mở map server, AMCL và RViz. Trong RViz:

1. Đặt **Global Options → Fixed Frame** thành `map`; có thể gõ trực tiếp
   nếu danh sách chưa có frame này.
2. Chọn **2D Pose Estimate**, nhấp vị trí hiện tại của robot trên bản đồ
   và kéo mũi tên theo hướng robot.
3. Đợi scan khớp với vật cản trên map.

Terminal 4 kiểm tra TF:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Khi thấy pose được xuất, `Ctrl+C` dừng riêng lệnh kiểm tra.
**Giữ localization và RViz chạy khi sang bước 3.**

### Bước 3 — Navigation

Terminal 3, sau khi teleop đã tắt:

```bash
ros2 launch robot_navigation navigation.launch.py
```

Đợi Nav2 chuyển sang `active`. Trong cửa sổ RViz của localization, chọn
**Nav2 Goal**, nhấp đích ở vùng trống đã lập bản đồ và kéo mũi tên đặt hướng
cuối. Robot sẽ lập kế hoạch và di chuyển đến đích.

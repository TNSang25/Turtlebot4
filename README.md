# TurtleBot4 — ROS 2 Humble / Gazebo Fortress

Mô phỏng TurtleBot4, xây dựng bản đồ bằng SLAM Toolbox, định vị bằng AMCL
và điều hướng bằng Nav2.

## 1. Cấu trúc chương trình

Các package nằm trực tiếp ở thư mục gốc repo GitHub:

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

Mỗi package có `package.xml` và `licenses/`. Bốn package đầu dùng
`CMakeLists.txt`; `robot_node` dùng `setup.py` và `setup.cfg`.

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

Đối với workspace mới, clone repo **vào thư mục `src/` chưa có nội dung**:

```bash
mkdir -p ~/workspaces/turtlebot4_ws
cd ~/workspaces/turtlebot4_ws
git clone https://github.com/TNSang25/Turtlebot4.git src
```

Sau khi clone, cấu trúc workspace là:

```text
turtlebot4_ws/
└── src/
    ├── README.md
    ├── robot_description/
    ├── robot_sim/
    ├── robot_mapping/
    ├── robot_navigation/
    └── robot_node/
```

Nếu workspace đã có các package, bỏ qua bước clone. Build tại thư mục gốc
workspace:

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install
source install/setup.bash
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

Đợi robot xuất hiện trong Gazebo. **Giữ simulation chạy suốt ba bước sau**.
Nếu muốn mở RViz cùng simulation, thêm `rviz:=true`. Camera RGB dùng topic
`/camera/image`; để xem trong RViz, thêm display **Image** và chọn topic này.

### Bước 1 — Mapping

Terminal 2 mở SLAM Toolbox và RViz:

```bash
ros2 launch robot_mapping mapping.launch.py
```

Terminal 3 mở teleop:

```bash
ros2 run robot_node teleop_keyboard
```

Focus terminal teleop, dùng các phím chữ thường:

| Phím | Chức năng |
| --- | --- |
| `w` / `s` | Tiến / lùi |
| `a` / `d` | Quay trái / phải |
| `Space` | Dừng |
| `Ctrl+C` | Dừng và thoát |

Lái robot quan sát các vùng cần lập bản đồ. Khi hoàn thành, bấm `Space` dừng
robot. Terminal 4 lưu map **khi mapping vẫn đang chạy**:

```bash
ros2 launch robot_mapping save_map.launch.py \
  map_name:=$PWD/src/robot_navigation/maps/my_map
```

Đợi thông báo **Map saved successfully**. Lệnh tạo `my_map.yaml` và
`my_map.pgm` trong `src/robot_navigation/maps/`. Đổi tên `my_map` nếu muốn
lưu một bản đồ khác; `map_name` không có đuôi file.

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

AMCL cần initial pose và scan hợp lệ để xuất TF `map → odom`. Robot đã di
chuyển khi mapping nên đặt pose hiện tại, không mặc định `(0, 0)`.

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

Khi navigation chạy, giữ đồng thời **simulation, localization và navigation**.
Không chạy teleop hoặc mapping trong bước này.

| Giai đoạn | Terminal 1 | Terminal 2 | Terminal 3 | Terminal 4 |
| --- | --- | --- | --- | --- |
| Mapping | Simulation | Mapping + RViz | Teleop | Lưu map |
| Localization | Simulation | Localization + RViz | Đã tắt teleop | Kiểm tra TF |
| Navigation | Simulation | Giữ localization + RViz | Navigation | Có thể để trống |

Khi kết thúc, hủy goal đang chạy nếu có; tắt navigation, đóng RViz, rồi tắt
localization và simulation.

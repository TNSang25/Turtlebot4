# TurtleBot4 — ROS 2 Humble / Gazebo Fortress

Workspace mô phỏng TurtleBot4 Standard với mô hình chính thức của TurtleBot4
và iRobot Create 3, SLAM Toolbox để xây dựng bản đồ, AMCL để định vị và Nav2
để điều hướng. Quy trình chạy: **simulation → mapping → localization → navigation**.

## 1. Cấu trúc chương trình

```text
Turtlebot4/                  # Thư mục gốc repo GitHub
├── README.md
├── robot_description/
│   ├── CMakeLists.txt, package.xml
│   ├── urdf/          # Xacro TurtleBot4, Create 3, cảm biến, adapter Gazebo
│   ├── meshes/        # 11 mesh DAE từ nhà phát triển
│   ├── config/        # simulation.yaml: DiffDrive và cảm biến
│   ├── launch/        # display.launch.py
│   ├── rviz/
│   └── licenses/
├── robot_sim/
│   ├── CMakeLists.txt, package.xml
│   ├── worlds/        # demo.sdf, empty.sdf
│   ├── maps/          # demo.yaml, demo.pgm: map mẫu sinh từ world
│   ├── config/        # bridge_config.yaml, demo_layout.yaml
│   ├── launch/        # sim, gazebo, spawn, bridge
│   ├── scripts/       # Sinh world/map và kiểm tra mô phỏng
│   └── licenses/
├── robot_mapping/
│   ├── CMakeLists.txt, package.xml
│   ├── config/        # slam.yaml
│   ├── launch/        # mapping.launch.py, save_map.launch.py
│   ├── rviz/
│   └── licenses/
├── robot_navigation/
│   ├── CMakeLists.txt, package.xml
│   ├── maps/          # my_map.yaml, my_map.pgm: map SLAM đã lưu
│   ├── config/        # nav2.yaml: AMCL, planner, controller, costmap
│   ├── launch/        # localization.launch.py, navigation.launch.py
│   ├── rviz/
│   └── licenses/
└── robot_node/
    ├── setup.py, setup.cfg, package.xml
    ├── robot_node/    # Teleop và xử lý vận tốc
    ├── resource/      # Marker đăng ký package Python
    ├── test/
    └── licenses/
```

| Package | Vai trò |
| --- | --- |
| `robot_description` | Mô tả hình học, khớp, frame và cấu hình mô phỏng robot. |
| `robot_sim` | Mở world, spawn robot và bridge ROS–Gazebo. |
| `robot_mapping` | Chạy SLAM Toolbox và lưu occupancy map. |
| `robot_navigation` | Lưu map, chạy localization và navigation bằng hai launch riêng. |
| `robot_node` | Node teleop phát lệnh `geometry_msgs/msg/Twist` trên `/cmd_vel`. |

Trên GitHub, 5 package nằm ngay ở thư mục gốc repo. Để chạy các lệnh trong
hướng dẫn, đặt nội dung repo vào `~/workspaces/turtlebot4_ws/src/`:

```text
turtlebot4_ws/
├── src/            # README.md và 5 thư mục robot_* của repo
├── build/          # colcon tạo khi build
├── install/        # Các package được cài đặt sau build
└── log/            # Log của colcon
```

Trong workspace, README chính vẫn ở `src/README.md`; giấy phép và attribution
nằm ở `licenses/` của từng package. `.gitkeep` chỉ giữ thư mục rỗng trong Git,
không tham gia runtime.

## 2. Cài dependency và build

Môi trường: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress.

```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard

cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install
source install/setup.bash
```

Nếu đã xóa `install/` và thấy cảnh báo prefix không tồn tại, mở terminal mới
hoặc bỏ các prefix cũ trước khi source ROS và build lại:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
```

## 3. Các bước chạy và hướng dẫn

### Chuẩn bị mỗi terminal

Mở 4 terminal. Trong **mỗi terminal**, chạy:

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
```

Các lệnh dùng `$PWD` và đường dẫn lưu map mặc định bên dưới giả định terminal
đang ở thư mục gốc `turtlebot4_ws`. Tất cả node phải dùng cùng ROS domain.

### Mở simulation — terminal 1

```bash
ros2 launch robot_sim sim.launch.py
```

Đợi Gazebo mở world `demo` và robot xuất hiện. **Giữ simulation chạy suốt
mapping, localization và navigation**, không khởi động lại giữa các bước.

### Bước 1 — Mapping và lưu map

Terminal 2 mở SLAM Toolbox cùng RViz:

```bash
ros2 launch robot_mapping mapping.launch.py
```

Terminal 3 mở teleop:

```bash
ros2 run robot_node teleop_keyboard
```

Bấm vào terminal teleop, dùng phím chữ thường `w/s` để tiến/lùi, `a/d` để
quay. Lái robot qua các vùng cần quan sát; xem map mở rộng trong RViz. Dùng
`Space` dừng robot khi muốn lưu bản đồ.

Terminal 4 lưu map **khi mapping vẫn đang chạy**:

```bash
ros2 launch robot_mapping save_map.launch.py
```

Đợi thông báo **Map saved successfully** và lệnh kết thúc. Kết quả mặc định:

```text
src/robot_navigation/maps/my_map.yaml
src/robot_navigation/maps/my_map.pgm
```

Để lưu với tên khác:

```bash
ros2 launch robot_mapping save_map.launch.py \
  map_name:=$PWD/src/robot_navigation/maps/room
```

`map_name` là đường dẫn **không có đuôi file**. Lệnh lưu mặc định có thể ghi đè
`my_map`; dùng tên khác để giữ nhiều bản đồ. Sau khi lưu xong, `Ctrl+C` tắt
**teleop ở terminal 3 và mapping ở terminal 2**. Giữ terminal 1 chạy.

### Bước 2 — Localization

Terminal 2:

```bash
ros2 launch robot_navigation localization.launch.py \
  map:=$PWD/src/robot_navigation/maps/my_map.yaml
```

Launch khởi động `map_server`, `amcl`, `lifecycle_manager_localization` và
RViz; các node planner/controller chưa chạy.

1. Trong RViz chọn **2D Pose Estimate**.
2. Nhấp vị trí hiện tại của robot trên map, kéo mũi tên theo hướng robot.
3. Đợi scan khớp với vật cản trên map và vị trí robot hiển thị đúng.

Robot đã di chuyển khi mapping nên pose ban đầu của AMCL cần là **pose hiện
tại**, không mặc định `(0, 0)`. Nếu đã khởi động lại simulation, đặt lại pose
đúng với vị trí spawn. Nếu dùng map tên `room`, thay đường dẫn bằng
`$PWD/src/robot_navigation/maps/room.yaml`.

Kiểm tra từ terminal 4:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 run tf2_ros tf2_echo map base_link
```

Hai node phải ở trạng thái `active`; TF phải xuất được pose. `Ctrl+C` dừng
riêng `tf2_echo`. **Giữ localization và RViz chạy khi sang bước 3.**

### Bước 3 — Navigation

Terminal 3, sau khi teleop đã tắt:

```bash
ros2 launch robot_navigation navigation.launch.py
```

Launch khởi động các node Nav2 và `lifecycle_manager_navigation`, dùng map/TF
từ localization. RViz mặc định tắt vì cửa sổ ở terminal 2 đang mở.

1. Đợi các node Nav2 chuyển sang `active`.
2. Trong RViz chọn **Nav2 Goal**, nhấp đích ở vùng trống đã được lập bản đồ
   rồi kéo mũi tên để đặt hướng cuối.
3. Quan sát path và robot di chuyển; theo dõi kết quả của tác vụ điều hướng.

Có thể kiểm tra bằng:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

Các node trên phải `active`. Simulation, localization và navigation phải
cùng chạy trong suốt quá trình điều hướng.

| Giai đoạn | Terminal 1 | Terminal 2 | Terminal 3 | Terminal 4 |
| --- | --- | --- | --- | --- |
| Mapping | Simulation | Mapping + RViz | Teleop | Lưu map |
| Localization | Simulation | Localization + RViz | Đã tắt teleop | Kiểm tra TF/lifecycle |
| Navigation | Simulation | Giữ localization + RViz | Navigation | Kiểm tra lifecycle |

Khi kết thúc, hủy goal đang chạy trong RViz nếu có, rồi `Ctrl+C` tắt navigation,
localization và cuối cùng simulation.

## 4. Launch và tùy chọn

| Launch | Chức năng |
| --- | --- |
| `robot_sim sim.launch.py` | Gọi Gazebo, spawn và bridge; RViz mặc định tắt. |
| `robot_sim gazebo.launch.py` | Chỉ mở world và Gazebo. |
| `robot_sim spawn.launch.py` | Chạy robot_state_publisher và spawn robot. |
| `robot_sim bridge.launch.py` | Chạy một parameter_bridge đọc bridge_config.yaml. |
| `robot_mapping mapping.launch.py` | SLAM Toolbox và RViz, dùng thời gian mô phỏng. |
| `robot_mapping save_map.launch.py` | Lưu YAML/PGM rồi thoát. |
| `robot_navigation localization.launch.py` | Map server, AMCL, lifecycle manager; RViz mặc định bật. |
| `robot_navigation navigation.launch.py` | Các node điều hướng và lifecycle manager; RViz mặc định tắt. |

```bash
ros2 launch robot_sim sim.launch.py world:=empty
ros2 launch robot_sim sim.launch.py headless:=true
ros2 launch robot_sim sim.launch.py enable_camera:=true
ros2 launch robot_sim sim.launch.py x:=1.0 y:=0.0 yaw:=1.57
```

`map:=...` thuộc `localization.launch.py`. `navigation.launch.py` không khởi
động AMCL/map server và không có đối số `localization`. Nếu localization chạy
với `rviz:=false`, có thể mở RViz bằng `navigation.launch.py rviz:=true`.

Để thử map mẫu của world `demo`, chạy localization không truyền `map`:

```bash
ros2 launch robot_navigation localization.launch.py
```

Nếu muốn điều hướng ngay khi SLAM đang chạy, giữ mapping, tắt teleop, bỏ qua
AMCL và chạy `navigation.launch.py rviz:=true`. Không chạy SLAM và AMCL đồng
thời cho cùng robot: chỉ một nguồn xuất TF `map → odom`.

## 5. Teleop và luồng điều khiển

| Phím | Lệnh |
| --- | --- |
| `w` / `s` | Tiến / lùi |
| `a` / `d` | Quay trái / phải |
| `q` / `e` | Tiến theo cung trái / phải |
| `z` / `c` | Lùi theo cung trái / phải |
| `Space` | Dừng |
| `Ctrl+C` | Dừng và thoát |

Teleop tự viết xuất `/cmd_vel` ở 20 Hz, mặc định 0,20 m/s và 0,80 rad/s;
gửi lệnh zero sau 0,6 giây không nhận phím. Khi giữ `w`, log cần hiện
`linear.x=0.20`. Dùng chữ thường; terminal teleop cần được focus.

```bash
ros2 run robot_node teleop_keyboard --ros-args \
  -p linear_speed:=0.15 -p angular_speed:=0.6
```

Teleop ROS chính thức dùng `i` tiến, `,` lùi, `j/l` quay và `k` dừng:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=false -p speed:=0.15 -p turn:=0.6 -r cmd_vel:=/cmd_vel
```

Chỉ chạy một nguồn điều khiển tại một thời điểm. Teleop tự viết vẫn phát
lệnh zero khi không nhận phím, nên phải tắt khi Nav2 điều khiển robot.

```text
Teleop / node điều khiển / Nav2
  → /cmd_vel (ROS Twist)
  → ros_gz_bridge/parameter_bridge
  → /cmd_vel (Ignition Twist)
  → Gazebo DiffDrive
```

Không có node watchdog hay topic `/cmd_vel_sim`. Khi thử publisher trực tiếp,
Gazebo giữ lệnh vận tốc cuối cho đến khi nhận lệnh mới; gửi zero để dừng:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.15}, angular: {z: 0.0}}'
# Ctrl+C dừng publisher, rồi gửi lệnh dừng:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

## 6. Map, SLAM và kiểm tra lỗi

Map SLAM để chạy AMCL nằm trong `robot_navigation/maps/`. Map mẫu của world
nằm trong `robot_sim/maps/`.
Lưu map vào source giữ dữ liệu khi xóa `build/` hoặc `install/`. `maps/` được
cài cùng package khi build; cần build lại sau khi thêm map nếu muốn dùng bản
trong `install/`. Quy trình ở mục 3 nạp trực tiếp map từ source.

YAML/PGM lưu occupancy grid, không chứa pose graph để tiếp tục một phiên SLAM.
Muốn tiếp tục graph, dùng dịch vụ serialize của SLAM Toolbox.

`robot_mapping/config/slam.yaml` dựa trên TurtleBot4 Humble: frame `base_link`,
`odom`, `map`, topic `/scan`, độ phân giải 0,05 m, ngưỡng LiDAR 0,165–12 m.
Ngưỡng dịch chuyển 0,05 m và góc 0,10 rad giảm scan trùng khi đứng yên; map
xuất mỗi 1 giây, loop closure tự động bật. SLAM Toolbox 2.6.10 có bộ lọc tịnh
tiến trước matcher, nên chỉ quay tại chỗ có thể chưa thêm scan; di chuyển thêm
khoảng 5 cm để nhận scan mới.

Bridge xuất `/clock`, `/odom`, `/tf`, `/joint_states`, `/scan`, `/imu`. Khi
bật camera, có thêm `/camera/image`, `/camera/depth_image`, `/camera/camera_info`.
Các node dùng `use_sim_time:=true` theo mặc định.

```text
map --(SLAM hoặc AMCL)--> odom --(DiffDrive)--> base_link
  → rplidar_link / imu_link / các frame camera và bánh xe
```

Nếu robot không di chuyển, kiểm tra simulation đang chạy, domain giống nhau,
teleop phát vận tốc khác zero và chỉ có một nguồn điều khiển:

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel
ros2 topic hz /scan
```

Nếu RViz báo scan queue full, kiểm tra chuỗi TF và thời gian mô phỏng. Nếu
TF đúng nhưng RViz vẫn lỗi, đóng riêng RViz, giữ SLAM và mở lại:

```bash
rviz2 -d "$(ros2 pkg prefix robot_mapping)/share/robot_mapping/rviz/mapping.rviz" \
  --ros-args -p use_sim_time:=true
```

Có thể chạy mapping với `rviz:=false` rồi mở RViz riêng bằng lệnh trên.

## 7. Kiểm tra mô phỏng

```bash
PYTHONNOUSERSITE=1 colcon test --packages-select robot_node
colcon test-result --verbose
# Khi simulation đang mở world demo:
ros2 run robot_sim smoke_test.py
ros2 run robot_sim smoke_test.py --drive
# Nếu bật camera:
ros2 run robot_sim smoke_test.py --camera
```

`--drive` gửi lệnh tiến ngắn rồi zero để dừng. Map `robot_sim/maps/demo.*`
được sinh từ hình học world. Nếu sửa `config/demo_layout.yaml`, tạo lại bằng:

```bash
python3 src/robot_sim/scripts/generate_demo_world.py
```

## 8. Mô hình chính thức và nguồn tham khảo

- [TurtleBot4 Humble, commit a6ee13b](https://github.com/turtlebot/turtlebot4/tree/a6ee13b63cbd524500cc6a68cd20dbef69f326a9/turtlebot4_description): Apache-2.0.
- [iRobot Create 3 Humble, commit 7d7a4f6](https://github.com/iRobotEducation/create3_sim/tree/7d7a4f69a503d4eef22f13aeb8755c86955b8cc5/irobot_create_common/irobot_create_description): BSD-3-Clause.
- [TurtleBot4 SLAM config](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/config/slam.yaml).
- [SLAM Toolbox Humble](https://github.com/SteveMacenski/slam_toolbox/tree/humble).
- [Nav2 Humble](https://github.com/ros-navigation/navigation2/tree/humble).
- [ros_gz_bridge Humble](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge).

11 mesh DAE giữ nguyên byte từ upstream. Xacro được include trực tiếp,
giữ hình học, inertia, collision và các khớp của nhà phát triển. Adapter
`turtlebot4.gazebo.xacro` cung cấp plugin/sensor runtime cho workspace này.
Firmware iRobot, docking và sự kiện IR/cliff/bumper chưa được triển khai.

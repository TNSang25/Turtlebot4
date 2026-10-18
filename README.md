# TurtleBot4 — Localization, Navigation tự động và YOLO

ROS 2 Humble / Gazebo Fortress. Sử dụng lại map SLAM đã lưu, định vị bằng
AMCL, tự động đi qua các goal bằng Nav2 và nhận diện vật thể từ camera RGB.

## 1. Cấu trúc chương trình

```text
Turtlebot4/                         # Thư mục gốc repo
├── README.md
├── robot_description/
│   ├── urdf/                      # TurtleBot4, Create 3, cảm biến và plugin
│   ├── meshes/                    # Mesh chính thức
│   ├── config/                    # simulation.yaml
│   ├── launch/
│   └── rviz/
├── robot_sim/
│   ├── worlds/                    # demo, empty, demo_objects
│   ├── models/person_standing/    # Model người 3D từ OSRF
│   ├── maps/                      # Map mẫu hình học của world demo
│   ├── config/                    # Bridge ROS–Gazebo
│   ├── launch/                    # sim, gazebo, spawn, bridge
│   └── scripts/
├── robot_mapping/
│   ├── config/                    # slam.yaml
│   ├── launch/                    # mapping, save_map
│   └── rviz/
├── robot_navigation/
│   ├── maps/                      # my_map.yaml + my_map.pgm đã xây dựng
│   ├── config/                    # nav2.yaml: AMCL, planner, controller
│   ├── launch/                    # localization, navigation
│   └── rviz/                      # Map, Publish Point, camera, ảnh YOLO
├── robot_node/
│   ├── robot_node/
│   │   ├── teleop_keyboard.py
│   │   ├── goal_recorder.py        # Ghi tọa độ khảo sát vào YAML
│   │   ├── goals.py                # Kiểm tra goal và khoảng trống trên map
│   │   └── waypoint_navigator.py   # Gửi tuần tự action /navigate_to_pose
│   ├── config/goals.yaml          # 3 goal mẫu cho demo; cần khảo sát lại
│   ├── launch/waypoints.launch.py
│   └── test/
└── robot_perception/
    ├── robot_perception/yolo_detector.py
    ├── config/yolo.yaml
    ├── launch/yolo.launch.py
    ├── models/                    # yolov8n.pt sau khi tải
    ├── scripts/download_model.py
    └── requirements.txt
```

| Package | Chức năng |
| --- | --- |
| `robot_description` | Mô tả robot, frame, cảm biến và cấu hình mô phỏng. |
| `robot_sim` | Mở world, spawn robot và bridge các topic ROS–Gazebo. |
| `robot_mapping` | SLAM Toolbox và lưu bản đồ khi cần tạo map mới. |
| `robot_navigation` | Nạp map đã lưu, chạy AMCL và Nav2 bằng các launch riêng. |
| `robot_node` | Teleop lúc khảo sát, ghi goal và chạy mission tự động. |
| `robot_perception` | Nhận ảnh, chuyển đổi bằng cv_bridge, chạy YOLO và xuất ảnh nhận diện. |

Mỗi package có `package.xml` và `licenses/`. `robot_node` và
`robot_perception` dùng `ament_python`; các package còn lại dùng `ament_cmake`.

## 2. Clone và cài đặt

Yêu cầu **Ubuntu 22.04, ROS 2 Humble** đã cài đặt. Cài dependency:

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-yaml python3-venv \
  python3-opencv ros-humble-cv-bridge \
  ros-humble-ros-gz ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup
```

Với workspace mới, clone repo vào thư mục `src/` chưa có nội dung:

```bash
mkdir -p ~/workspaces/turtlebot4_ws
cd ~/workspaces/turtlebot4_ws
git clone https://github.com/TNSang25/Turtlebot4.git src
```

Nếu workspace đã có các package, bỏ qua bước clone. Sau đó cài YOLO trong
môi trường Python riêng; NumPy được giới hạn ở 1.x để dùng với cv_bridge Humble:

```bash
cd ~/workspaces/turtlebot4_ws
python3 -m venv --system-site-packages .venv-yolo
.venv-yolo/bin/python -m pip install torch torchvision \
  --index-url https://download.pytorch.org/whl/cpu
.venv-yolo/bin/python -m pip install -r src/robot_perception/requirements.txt
python3 src/robot_perception/scripts/download_model.py

source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install
source install/setup.bash
```

Không cần activate venv khi chạy các node ROS; launch YOLO nhận đường dẫn
interpreter ở bước chạy bên dưới. Trọng số tải một lần từ Ultralytics,
có kiểm tra SHA256; không tải model trong quá trình robot di chuyển.

Model mặc định là [YOLOv8n](https://docs.ultralytics.com/models/yolov8/), dùng
[các class COCO](https://docs.ultralytics.com/datasets/detect/coco/) như
`person`, `car`, `chair`. COCO không có class bình chữa cháy; muốn nhận diện
class này cần model đã huấn luyện phù hợp và truyền `model_path:=.../best.pt`.

## 3. Các bước chạy

### Chuẩn bị mỗi terminal

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
```

Các lệnh bên dưới chạy từ thư mục `turtlebot4_ws`. Sử dụng bản đồ đã xây dựng
ở `src/robot_navigation/maps/my_map.yaml` cùng ảnh `my_map.pgm`. Nếu map của
bạn tên `map.yaml`, thay đường dẫn tương ứng; giữ YAML và ảnh mà nó tham chiếu
trong cùng thư mục. Không chạy SLAM cùng AMCL trong bài sử dụng lại map này.

### Bước 1 — Simulation trong đúng world

Terminal 1:

```bash
ros2 launch robot_sim sim.launch.py world:=demo_objects enable_camera:=true
```

`demo_objects` giữ nguyên tường và vật cản của world `demo` đã dùng lập map,
thêm ba model người 3D từ OSRF. Map cũ không được tạo lại. Nếu tuần trước dùng
world khác, truyền đúng world đó bằng `world:=...`; dùng `world:=demo` để
chạy phòng gốc. Giữ simulation chạy suốt phiên.

### Bước 2 — Nạp map và localization

Terminal 2:

```bash
ros2 launch robot_navigation localization.launch.py \
  map:=$PWD/src/robot_navigation/maps/my_map.yaml
```

Trong RViz đặt **Global Options → Fixed Frame = map** (có thể gõ trực tiếp),
rồi dùng **2D Pose Estimate** đặt vị trí và hướng hiện tại của robot. Kiểm tra
LiDAR `/scan` trùng các tường trên bản đồ. Nếu biết pose trong map, có thể
truyền `set_initial_pose:=true` cùng `initial_pose_x:=...`, `initial_pose_y:=...`
và `initial_pose_yaw:=...` khi launch; yaw dùng radian, không mặc định robot ở
`(0, 0)` nếu nó đã di chuyển hoặc map khác gốc.

Terminal phụ kiểm tra:

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /map_server
ros2 run tf2_ros tf2_echo map base_link
```

Hai node phải `active` và TF xuất pose. Dừng riêng `tf2_echo` bằng `Ctrl+C`;
**giữ localization và RViz chạy**.

### Bước 3 — Khảo sát và ghi ít nhất 3 goal

Terminal 3 mở công cụ ghi goal:

```bash
ros2 run robot_node goal_recorder --ros-args -p use_sim_time:=true \
  -p output_file:=$PWD/src/robot_navigation/maps/goals.yaml
```

Trong RViz chọn **Publish Point** và nhấp ít nhất ba vị trí trống thuộc các
khu vực khác nhau. Tọa độ `(x, y)` được ghi từ `/clicked_point`; yaw mặc định
là 0 rad. Có thể đổi yaw trước lần nhấp tiếp theo:

```bash
ros2 param set /goal_recorder clicked_yaw 1.5708
ros2 topic echo /clicked_point
```

Hoặc lái robot bằng `ros2 run robot_node teleop_keyboard` **trong giai đoạn
khảo sát**, dừng ở từng điểm và gọi:

```bash
ros2 service call /record_current_pose std_srvs/srv/Trigger '{}'
```

Dịch vụ ghi cả `(x, y, yaw)` hiện tại từ TF `map → base_link`. Chờ TF có dữ
liệu rồi mới gọi; nếu dịch vụ báo chưa có transform, đợi và gọi lại. Có thể
quan sát thêm bằng `ros2 topic echo /amcl_pose`.

File ghi nhận có dạng:

```yaml
frame_id: map
goals:
  - {name: Goal 1, x: -1.5, y: 0.0, yaw: 0.0}
  - {name: Goal 2, x: -1.5, y: 2.5, yaw: 1.5708}
  - {name: Goal 3, x: 2.5, y: 2.5, yaw: 0.0}
```

Đây là ví dụ cho map demo, không thay thế tọa độ tự khảo sát. Kiểm tra/chỉnh
`goals.yaml` để có ít nhất ba vị trí khác nhau, yaw đúng hướng mong muốn,
trong vùng đã biết và đủ khoảng trống cho robot. Recorder thêm vào file đã
có; chọn tên file mới nếu muốn ghi một bộ goal mới.

Sau khi khảo sát, `Ctrl+C` tắt **teleop và goal_recorder**.

### Bước 4 — Khởi động Nav2

Terminal 3:

```bash
ros2 launch robot_navigation navigation.launch.py
```

Đợi các node `planner_server`, `controller_server`, `bt_navigator` chuyển
sang `active`. Giữ simulation, localization và navigation cùng chạy.

### Bước 5 — Nhận diện YOLO từ camera

Terminal 4:

```bash
ros2 launch robot_perception yolo.launch.py \
  python_executable:=$PWD/.venv-yolo/bin/python \
  model_path:=$PWD/src/robot_perception/models/yolov8n.pt
```

Node subscribe `/camera/image` kiểu `sensor_msgs/msg/Image`, chuyển sang BGR
bằng `cv_bridge`, chạy YOLO và xuất:

| Topic | Nội dung |
| --- | --- |
| `/detections/image` | Ảnh có bounding box, tên class và độ tin cậy. |
| `/detections/objects` | `std_msgs/msg/String` chứa JSON: timestamp, frame, class, confidence và bbox xyxy theo pixel ảnh gốc. |

Trong RViz bật **YOLO Detections**, hoặc **Add → Image** rồi chọn
`/detections/image`, QoS **Best Effort / Volatile**. Terminal in class và
confidence; ảnh không có detection vượt ngưỡng vẫn được xuất.
Để mở cửa sổ OpenCV, thêm `show_window:=true`; `Ctrl+C` dừng node.

Trong kiểm thử có phát hiện đúng `person` và cũng có nhãn sai thoáng qua trên
chi tiết của scene. Model COCO chưa được tinh chỉnh cho Gazebo; cần xem ảnh,
confidence và dùng dữ liệu phù hợp để đánh giá độ chính xác.

Camera trong workspace dùng `/camera/image`. Nếu dùng topic khác, truyền
`image_topic:=/oakd/rgb/preview/image_raw`. Cấu hình tốc độ inference, confidence,
CPU threads và device nằm trong `robot_perception/config/yolo.yaml`.

### Bước 6 — Chạy chương trình tự động qua ba goal

Terminal 5:

```bash
ros2 launch robot_node waypoints.launch.py \
  goals_file:=$PWD/src/robot_navigation/maps/goals.yaml
```

Node `waypoint_navigator` chờ map, TF và Nav2 active, kiểm tra footprint tại
các goal, rồi gửi từng action `/navigate_to_pose`. Chỉ khi Goal 1 trả
`SUCCEEDED` mới gửi Goal 2, tiếp đó Goal 3. Dừng 2 giây tại mỗi điểm để quan
sát camera. Nav2 lập kế hoạch và tránh vật cản bằng costmap/LiDAR; YOLO cung
cấp kết quả nhận diện độc lập.

**Từ lúc chạy node này, không bật teleop và không dùng chuột đặt Nav2 Goal.**
Nếu một goal bị từ chối, thất bại hoặc quá thời gian, chuỗi dừng; `Ctrl+C`
yêu cầu hủy goal đang chạy và gửi lệnh dừng. Có thể theo dõi trạng thái và
lưu kết quả:

```bash
ros2 topic echo /mission/status
# Hoặc thêm khi launch waypoints:
# result_file:=$PWD/mission_results.json
```

Để thử nhanh ba goal mẫu đã có trong package (chỉ phù hợp map demo hiện tại):

```bash
ros2 launch robot_node waypoints.launch.py
```

Kết thúc phiên: dừng node mission, node YOLO, Nav2, đóng RViz, rồi tắt
localization và simulation.

### Khi cần tạo lại map cho world khác

Chạy `robot_mapping mapping.launch.py` cùng teleop trong giai đoạn lập map.
Khi SLAM vẫn đang chạy, lưu bằng:

```bash
ros2 launch robot_mapping save_map.launch.py \
  map_name:=$PWD/src/robot_navigation/maps/my_map
```

Đợi lưu thành công rồi tắt mapping/teleop, sau đó quay lại bước localization.

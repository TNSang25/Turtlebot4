# Điều khiển TurtleBot 4 bằng cử chỉ tay

`gesture_recognition_node` đọc webcam bằng OpenCV, chạy MediaPipe và publish
`/detected_gesture` (`std_msgs/msg/String`). `robot_behavior_node` trong
`robot_node` nhận cử chỉ, điều khiển `/cmd_vel` và xuất `/robot_behavior/state`.

## Cử chỉ

| Cử chỉ / giá trị topic | Tác dụng |
| --- | --- |
| Vẫy tay / `Wave` | Chọn động tác chào: xoay trái, phải rồi về hướng ban đầu. |
| Giơ 1 ngón / `Pointing_Up` | Chọn tiến–lùi 4 lần, mỗi chặng khoảng 0,35 m. |
| Giơ 2 ngón / `Victory` | Chọn đi một vòng tròn, bán kính 0,60 m. |
| Ngón cái lên / `Thumb_Up` | Xác nhận và thực hiện hành vi đã chọn. |
| Xòe tay / `Open_Palm` | Dừng ngay. Hành vi đang chạy không tự tiếp tục. |
| Nắm tay / `Closed_Fist` | Hủy hành vi và xóa lựa chọn, trở về chờ. |

MediaPipe có sẵn 5 cử chỉ tĩnh trên. `Wave` được nhận biết thêm từ chuyển động
sang hai bên của bàn tay xòe, với ít nhất hai lần đổi hướng; không phải một
class AI có sẵn. Tham khảo [MediaPipe Gesture Recognizer](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer).

Nếu model trả `None` hoặc độ tin cậy thấp, node kiểm tra thêm góc khớp và độ
duỗi ngón tay từ landmark 3D cho cử chỉ chọn, dừng và hủy. `Thumb_Up` xác nhận
vẫn phải do model nhận diện đủ ngưỡng. Cửa sổ hiển thị riêng `Model` (nhãn và
độ tin cậy gốc), `pose` (tư thế hình học), `Gesture` (kết quả gửi cho robot).

## Cài đặt

Trong workspace đã có các package và dependency ROS theo README chính:

```bash
cd ~/workspaces/turtlebot4_ws
sudo apt install python3-venv python3-pip libportaudio2
python3 -m venv --system-site-packages .venv-gesture
.venv-gesture/bin/python -m pip install -r src/robot_gesture/requirements.txt
python3 src/robot_gesture/scripts/download_model.py

source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select robot_sim robot_node robot_gesture
```

MediaPipe dùng venv riêng với YOLO. Model được tải trước, có kiểm tra SHA256.

## Thứ tự chạy

Chuẩn bị **mỗi terminal**:

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
```

**1. Terminal 1 — Simulation:** vùng giữa phòng demo đủ rộng cho các quỹ đạo mặc định.

```bash
ros2 launch robot_sim sim.launch.py world:=demo enable_camera:=false
```

**2. Terminal 2 — Webcam và hai node điều khiển:**

```bash
ros2 launch robot_gesture gesture_control.launch.py \
  python_executable:=$PWD/.venv-gesture/bin/python \
  model_path:=$PWD/src/robot_gesture/models/gesture_recognizer.task
```

Đổi `camera_index:=1` nếu webcam không nằm ở index 0. Thêm
`control_robot:=false` để chỉ thử webcam/nhận diện, không chạy node điều khiển.

**3. Demo bằng tay:**

1. Đưa một tay vào khung hình; giữ cử chỉ chọn khoảng 0,5 giây.
2. Chờ cửa sổ hiện `Selected`, rồi giữ Thumbs Up để xác nhận.
3. Giữ tay với một cử chỉ hợp lệ trong khung hình khi robot đang chạy.
4. Xòe tay để dừng hoặc nắm tay để hủy; chọn và xác nhận lại nếu muốn chạy tiếp.

Giơ 1 ngón: duỗi ngón trỏ, gập các ngón còn lại. Giơ 2 ngón: duỗi ngón trỏ
và ngón giữa thành chữ V. Nắm tay: gập cả ngón cái. Vẫy tay: xòe tay, đưa
các đầu ngón sang trái–phải–trái trong khoảng 2,5 giây; có thể lắc cổ tay.
Giữ trọn bàn tay trong ảnh. `Unknown` thoáng qua lúc đổi cử chỉ là bình thường;
nếu kéo dài, xem dòng `Model` và `pose` để phân biệt chưa thấy tay hay chưa
khớp tư thế. Dùng `control_robot:=false` để thử nhận diện trước.

Giữ một cử chỉ không kích hoạt lặp hành vi. Lựa chọn mới trong khi robot
đang chạy được bỏ qua; hai cử chỉ dừng/hủy luôn được ưu tiên.

Node dừng khi mất tay/cử chỉ hợp lệ khoảng 0,40 giây, mất bản tin 0,75 giây,
thiếu dữ liệu LiDAR/odom hoặc vật cản quá gần. Vòng tròn chỉ được bắt đầu khi
LiDAR cho thấy đủ vùng trống. Thông số nằm trong `config/gesture_control.yaml`.

Không chạy teleop, Nav2 hoặc node khác gửi `/cmd_vel` cùng lúc. Không cần
bàn phím để điều khiển robot trong demo; đóng cửa sổ webcam để kết thúc.

Terminal phụ xem dữ liệu:

```bash
ros2 topic echo /detected_gesture
ros2 topic echo /robot_behavior/state
ros2 topic echo /cmd_vel
```

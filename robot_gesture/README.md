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

**1. Terminal 1 — Simulation:** world `arena` có sàn trống 20 × 20 m và bốn
tường bao, robot ở giữa để thử các hành vi.

```bash
ros2 launch robot_sim sim.launch.py world:=arena enable_camera:=false
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

1. Đưa một tay vào khung hình, chờ cửa sổ hiện `Selected`.
2. Đổi sang Thumbs Up để xác nhận. Lựa chọn không hết hạn trong lúc chờ,
   kể cả khi xuất hiện `Unknown` hoặc `No_Hand`.
3. Giữ tay với một cử chỉ hợp lệ trong khung hình khi robot đang chạy.
4. Xòe tay để dừng hoặc nắm tay để hủy; chọn và xác nhận lại nếu muốn chạy tiếp.

Giơ 1 ngón: duỗi ngón trỏ, gập các ngón còn lại. Giơ 2 ngón: duỗi ngón trỏ
và ngón giữa thành chữ V. Nắm tay: gập cả ngón cái. Vẫy tay: xòe tay, đưa
các đầu ngón sang trái–phải–trái trong khoảng 2,5 giây; có thể lắc cổ tay.
Giữ trọn bàn tay trong ảnh. `Unknown` thoáng qua lúc đổi cử chỉ là bình thường;
nếu kéo dài, xem dòng `Model` và `pose` để phân biệt chưa thấy tay hay chưa
khớp tư thế. Dùng `control_robot:=false` để thử nhận diện trước.

Node lưu sự kiện cử chỉ đã chấp nhận (`Accepted`), kết hợp với lựa chọn đang
chờ để xử lý cử chỉ tiếp theo. Hai bản tin nhận diện liên tiếp tạo một sự kiện;
không áp thêm bộ đếm thời gian xác nhận. Giữ một cử chỉ không kích hoạt lặp
hành vi. Có thể thay lựa chọn bằng cử chỉ chọn khác, hoặc nắm tay để xóa.
Lựa chọn mới trong khi robot đang chạy được bỏ qua; dừng/hủy luôn được ưu tiên.

Trong lúc chạy, node dừng và yêu cầu chọn lại khi mất tay/cử chỉ hợp lệ khoảng
0,40 giây, mất bản tin 0,75 giây, thiếu LiDAR/odom hoặc vật cản quá gần. Khi
chỉ đang chờ xác nhận, lựa chọn vẫn được giữ. Bộ lọc nhận diện vẫn lọc nhiễu
ngắn trước khi gửi bản tin.

Nếu `Running` chưa có hành vi, xem dòng `Reason`: thiếu cảm biến, TF hoặc
không đủ khoảng trống. Với bán kính 0,60 m, đi tròn cần khoảng trống ít nhất
1,60 m quanh robot. Xác nhận bị chặn không xóa lựa chọn; sau khi xử lý nguyên
nhân, xòe tay rồi Thumbs Up để xác nhận lại. Giữ nguyên Thumbs Up không tự chạy
lại khi điều kiện thay đổi. Thông số nằm trong `config/gesture_control.yaml`.

Không chạy teleop, Nav2 hoặc node khác gửi `/cmd_vel` cùng lúc. Không cần
bàn phím để điều khiển robot trong demo; đóng cửa sổ webcam để kết thúc.

Terminal phụ xem dữ liệu:

```bash
ros2 topic echo /detected_gesture
ros2 topic echo /robot_behavior/state
ros2 topic echo /cmd_vel
```

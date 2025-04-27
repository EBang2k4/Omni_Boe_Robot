# Hướng dẫn 



---
---

## Lưu ý

- Đảm bảo đường dẫn các file map.yaml trong Omni_Diff_Robot/omni_diff/maps được sửa thành đường dẫn đúng của máy.
- Đảm bảo đường dẫn new_room.world đúng. Vào new_room.world, "CTRL+F: bang", chuột phải chọn Change All Ocurrence sau đó sửa thành đường dẫn chuẩn trong máy.
- Tải đủ thư viện Hector và Karto.

---
## Yêu cầu

**Laptop:**
- Ubuntu 20.04 với ROS Noetic (Cài đặt ROS Noetic).

**Workspace:**  
- Không gian làm việc ROS (ví dụ: `~/catkin_ws`).

---

## Cài đặt

### Clone Repository
Trên **cả laptop và Raspberry Pi 3**, clone package vào workspace ROS:

```bash
cd ~/catkin_ws/src
git clone https://github.com/EBang2k4/Omni_Diff_Robot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```


### Chạy SLAM

- CHỌN MODEL CHO ROBOT `omni` HOẶC `boe_bot`

```bash
export OMNI_DIFF_MODEL=MODEL_TYPE
```

- Chạy KARTO SLAM:

```bash
roslaunch omni_diff_slam omni_diff_karto_slam.launch 
```
- Chạy HECTOR SLAM:

```bash
roslaunch omni_diff_slam omni_diff_hector_slam.launch 
```

- Chạy Teleop_Node để quét map:

```bash
rosrun omni_diff_teleop omni_diff_teleop_key 
```

- Lưu map sau khi quét bản đồ:

```bash
rosrun map_server map_saver -f ~/catkin_ws/src/Omni_Diff_Robot/omni_diff/maps/TÊN_MAP
```

### Chạy NAVIGATION

- CHỌN MODEL CHO ROBOT `omni` HOẶC `boe_bot`

```bash
export OMNI_DIFF_MODEL=MODEL_TYPE
```

- Chạy NAVIGATION:

```bash
roslaunch omni_diff_navigation navigation.launch 
```
### Chạy NAVIGATION

- CHỌN MODEL CHO ROBOT `omni` HOẶC `boe_bot`

```bash
export OMNI_DIFF_MODEL=MODEL_TYPE
```

- Chạy HUMAN_TRACKING:
- CHỌN MODEL CHO ROBOT `omni` HOẶC `boe_bot`

```bash
export OMNI_DIFF_MODEL=MODEL_TYPE
```

```bash
roslaunch omni_diff_human_tracking yolo_tracker.launch 
```


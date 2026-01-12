# drone_gazebo_simulator
# Drone Gazebo Simulator (ROS 2 Jazzy)
## 📌 Deskripsi
**Drone Gazebo Simulator** adalah proyek simulasi drone berbasis **ROS 2 Jazzy** dan **Gazebo** yang berjalan pada **Ubuntu 22.04**.  
Simulator ini digunakan untuk menguji sistem kontrol drone dan Behaviour Tree.

📺 **Video Demo:**  
### Video Mapping Turtlebot4
[![Video](https://img.youtube.com/vi/kIQf73uS0oI/hqdefault.jpg)](https://www.youtube.com/watch?v=kIQf73uS0oI)

---

## 📁 Struktur Folder
drone_gazebo_simulator/
├── drone_behavior/ # Behavior Tree dan logika otonom
│ ├── config/
│ ├── src/
│ ├── include/
│ └── package.xml
│
├── drone_controller/ # Kontrol PID dan motion control
│ ├── src/
│ ├── include/
│ └── package.xml
│
├── launch/ # ROS 2 launch files
├── params/ # File parameter (.yaml)
└── README.md

---

## Fitur Utama
- ✈️ Simulasi drone di Gazebo
- 🎮 Kontrol PID untuk stabilisasi
- 🌳 Behavior Tree untuk pengambilan keputusan

---

## 🛠️ Kebutuhan Sistem
Pastikan sistem telah terinstal:
- Ubuntu 22.04
- ROS 2 Jazzy
- Gazebo
- colcon
- C++17 / Python 3

Instal ROS 2 Jazzy (ringkas):
```bash
sudo apt update
sudo apt install ros-jazzy-desktop
📦 Instalasi
Buat workspace ROS 2:

bash
Copy code
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
Clone repository:

bash
Copy code
git clone <repository-url>
Build workspace:

bash
Copy code
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
▶️ Menjalankan Simulator
1. Jalankan Gazebo
bash
Copy code
ros2 launch drone_sim gazebo_launch.py
2. Jalankan kontrol drone
bash
Copy code
ros2 launch drone_controller controller_launch.py
3. Jalankan Behavior Tree
bash
Copy code
ros2 launch drone_behavior behavior_launch.py
🧠 Behavior Tree
Behavior Tree digunakan untuk mengatur perilaku drone seperti:

Takeoff

Navigasi waypoint

Hover

Return to Home

Monitoring status

File konfigurasi:

arduino
Copy code
drone_behavior/config/behavior_tree_drone.xml
🔧 Topik ROS 2
Beberapa topic utama yang digunakan:

/cmd_vel — perintah kecepatan

/odom — posisi dan orientasi drone

/imu — data IMU

/battery_state — status baterai

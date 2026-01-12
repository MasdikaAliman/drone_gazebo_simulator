# drone_gazebo_simulator
# 🚁 Drone Gazebo Simulator (ROS 2 Jazzy)

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo-Simulator-orange" alt="Gazebo">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-purple?logo=ubuntu" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/C++-17-00599C?logo=cplusplus" alt="C++17">
  <img src="https://img.shields.io/badge/Python-3.10-3776AB?logo=python" alt="Python 3">
</p>

---

## 📌 Deskripsi

**Drone Gazebo Simulator** adalah proyek simulasi drone berbasis **ROS 2 Jazzy** dan **Gazebo** yang berjalan pada **Ubuntu 22.04**. Simulator ini digunakan untuk menguji sistem kontrol drone, Behaviour Tree, dan algoritma navigasi otonom dalam lingkungan virtual yang aman.

### 🎯 Kegunaan:
- 🔬 Pengembangan dan testing sistem kontrol drone
- 🤖 Implementasi Behavior Tree untuk logika otonom
- 🧪 Eksperimen algoritma navigasi dan path planning
- 🛡️ Testing tanpa risiko kerusakan hardware
- 🎓 Media pembelajaran robotika dan autonomous systems

---

## 📺 Video Demo

### Drone Simulation Demo
[![Video](https://img.youtube.com/vi/kIQf73uS0oI/hqdefault.jpg)](https://www.youtube.com/watch?v=kIQf73uS0oI)

*Klik gambar di atas untuk menonton video demo*

---

## 📁 Struktur Folder

```
drone_gazebo_simulator/
├── drone_behavior/          # Behavior Tree dan logika otonom
│   ├── config/              # Konfigurasi Behavior Tree (XML)
│   ├── src/                 # Source code behavior nodes
│   ├── include/             # Header files
│   └── package.xml          # Package metadata
│
├── drone_controller/        # Kontrol PID dan motion control
│   ├── src/                 # Source code controller
│   ├── include/             # Header files
│   └── package.xml          # Package metadata
│
├── drone_sim/               # Gazebo simulation files
│   ├── models/              # Model 3D drone (URDF/SDF)
│   ├── worlds/              # Environment simulasi
│   └── meshes/              # 3D mesh files
│
├── launch/                  # ROS 2 launch files
│   ├── gazebo_launch.py
│   ├── controller_launch.py
│   └── behavior_launch.py
│
├── params/                  # File parameter (.yaml)
│   └── drone_params.yaml
│
└── README.md
```

---

## ✨ Fitur Utama

- ✈️ **Simulasi Drone di Gazebo** - Environment 3D realistis dengan fisika akurat
- 🎮 **Kontrol PID** - Stabilisasi dan kontrol posisi drone
- 🌳 **Behavior Tree** - Sistem pengambilan keputusan modular
- 📡 **Multi Sensor** - IMU, GPS, dan sensor lainnya
- 🗺️ **Waypoint Navigation** - Navigasi otomatis ke target koordinat
- 🔋 **Battery Monitoring** - Simulasi konsumsi dan monitoring baterai
- 🏠 **Return to Home** - Fitur RTH otomatis

---

## 🛠️ Kebutuhan Sistem

### Sistem Operasi
- **Ubuntu 22.04 LTS** (Jammy Jellyfish)

### Software yang Diperlukan
- **ROS 2 Jazzy Jalisco**
- **Gazebo Classic** (gz-sim)
- **Python 3.10+**
- **C++17** compiler
- **colcon** build tools

### Instalasi ROS 2 Jazzy

```bash
# Update repository
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Jazzy Desktop
sudo apt install ros-jazzy-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions -y
sudo apt install ros-jazzy-gazebo-ros-pkgs -y
```

### Instalasi Dependencies Tambahan

```bash
# Install library yang dibutuhkan
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-controller-manager \
  ros-jazzy-tf2-tools \
  ros-jazzy-rqt* \
  python3-rosdep
```

---

## 📦 Instalasi Proyek

### 1️⃣ Buat Workspace ROS 2

```bash
# Buat workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2️⃣ Clone Repository

```bash
# Clone dari GitHub
git clone https://github.com/MasdikaAliman/drone_gazebo_simulator.git
```

### 3️⃣ Install Dependencies dengan rosdep

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4️⃣ Build Workspace

```bash
# Build semua packages
cd ~/ros2_ws
colcon build --symlink-install

# Source environment
source install/setup.bash
```

### 5️⃣ Tambahkan ke .bashrc (Opsional)

```bash
# Agar tidak perlu source setiap kali buka terminal
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## ▶️ Menjalankan Simulator

### 🚀 Quick Start (All-in-One)

```bash
# Jalankan semua komponen sekaligus
ros2 launch drone_sim full_simulation.launch.py
```

### 🔧 Manual Start (Step by Step)

#### 1. Jalankan Gazebo + Model Drone

```bash
# Terminal 1
ros2 launch drone_sim gazebo_launch.py
```

#### 2. Jalankan Drone Controller

```bash
# Terminal 2
ros2 launch drone_controller controller_launch.py
```

#### 3. Jalankan Behavior Tree

```bash
# Terminal 3
ros2 launch drone_behavior behavior_launch.py
```

---

## 🧠 Behavior Tree

Behavior Tree digunakan untuk mengatur perilaku drone secara modular dan reusable.

### Perilaku yang Tersedia:
- ✅ **Takeoff** - Lepas landas otomatis ke ketinggian tertentu
- ✅ **Waypoint Navigation** - Terbang ke koordinat yang ditentukan
- ✅ **Hover** - Mempertahankan posisi di udara
- ✅ **Return to Home (RTH)** - Kembali ke posisi awal
- ✅ **Landing** - Pendaratan aman
- ✅ **Battery Monitoring** - Cek status baterai real-time
- ✅ **Emergency Stop** - Stop darurat saat kondisi kritis

### Lokasi File Konfigurasi

```
drone_behavior/config/behavior_tree_drone.xml
```

### Contoh Struktur Behavior Tree

```xml
<root>
  <BehaviorTree ID="DroneMission">
    <Sequence>
      <Action ID="CheckBattery" threshold="20.0"/>
      <Action ID="Takeoff" height="2.0"/>
      <Action ID="GoToWaypoint" x="5.0" y="5.0" z="2.0"/>
      <Action ID="Hover" duration="5.0"/>
      <Action ID="GoToWaypoint" x="10.0" y="10.0" z="2.0"/>
      <Action ID="ReturnToHome"/>
      <Action ID="Land"/>
    </Sequence>
  </BehaviorTree>
</root>
```

---

## 🔧 Topik ROS 2

### 📡 Topics Utama

| Topic | Message Type | Deskripsi |
|-------|-------------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Perintah kecepatan drone (linear & angular) |
| `/odom` | `nav_msgs/Odometry` | Posisi dan orientasi drone saat ini |
| `/imu` | `sensor_msgs/Imu` | Data IMU (accelerometer, gyroscope) |
| `/battery_state` | `sensor_msgs/BatteryState` | Status baterai (voltage, percentage) |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Data GPS koordinat |
| `/camera/image_raw` | `sensor_msgs/Image` | Stream kamera drone |
| `/behavior_status` | `std_msgs/String` | Status behavior tree saat ini |

### 🔌 Services

| Service | Type | Deskripsi |
|---------|------|-----------|
| `/takeoff` | `std_srvs/Trigger` | Trigger takeoff |
| `/land` | `std_srvs/Trigger` | Trigger landing |
| `/set_waypoint` | `drone_msgs/SetWaypoint` | Set target waypoint |
| `/emergency_stop` | `std_srvs/Trigger` | Stop darurat |

### Monitoring Topics

```bash
# List semua topics aktif
ros2 topic list

# Monitor data topic tertentu
ros2 topic echo /odom

# Cek frekuensi publish
ros2 topic hz /cmd_vel

# Info detail topic
ros2 topic info /battery_state
```

---

## ⚙️ Konfigurasi Parameter

File konfigurasi utama: `params/drone_params.yaml`

```yaml
drone_controller:
  ros__parameters:
    # PID Tuning
    pid_pos_kp: 1.0
    pid_pos_ki: 0.0
    pid_pos_kd: 0.5
    
    pid_vel_kp: 0.8
    pid_vel_ki: 0.1
    pid_vel_kd: 0.3
    
    # Physical Limits
    max_velocity: 5.0       # m/s
    max_acceleration: 2.0   # m/s²
    max_tilt_angle: 30.0    # degrees
    
    # Control Loop
    update_rate: 50.0       # Hz

drone_behavior:
  ros__parameters:
    # Mission Parameters
    takeoff_height: 2.0     # meters
    landing_speed: 0.5      # m/s
    waypoint_tolerance: 0.3 # meters
    
    # Safety Parameters
    battery_threshold: 20.0 # percentage
    max_flight_time: 600.0  # seconds
    geofence_radius: 100.0  # meters
```

---

## 🎮 Kontrol Manual

### Keyboard Teleop

```bash
# Install teleop keyboard
sudo apt install ros-jazzy-teleop-twist-keyboard

# Jalankan teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/drone/cmd_vel
```

**Tombol Kontrol:**
- `i` / `k` - Maju / Mundur
- `j` / `l` - Kiri / Kanan  
- `u` / `o` - Naik / Turun
- `q` / `z` - Rotasi kiri / kanan
- `Space` - Stop

### Joystick Control

```bash
ros2 launch drone_sim joystick_control.launch.py
```

---

## 📊 Monitoring & Debugging

### Visualisasi di RViz2

```bash
# Launch RViz dengan konfigurasi drone
ros2 launch drone_sim rviz_display.launch.py
```

### RQT Tools

```bash
# Node graph visualization
rqt_graph

# Topic monitor
rqt_topic

# Plot data real-time
rqt_plot /odom/pose/pose/position/z
```

### TF Tree Visualization

```bash
# Lihat transformasi koordinat
ros2 run tf2_tools view_frames

# Monitor TF real-time
ros2 run rqt_tf_tree rqt_tf_tree
```

---

## 🧪 Testing

### Run Unit Tests

```bash
cd ~/ros2_ws
colcon test --packages-select drone_controller drone_behavior
colcon test-result --verbose
```

### Test Mission Script

```bash
# Jalankan test mission otomatis
ros2 launch drone_sim test_mission.launch.py
```

---

## 🐛 Troubleshooting

### ❌ Gazebo tidak muncul

```bash
# Kill proses yang crash
killall gzserver gzclient

# Clear cache
rm -rf ~/.gazebo/log/*

# Restart Gazebo
ros2 launch drone_sim gazebo_launch.py
```

### ❌ Drone jatuh / tidak stabil

**Solusi:**
1. Cek parameter PID di `params/drone_params.yaml`
2. Pastikan controller node running: `ros2 node list`
3. Monitor topic `/cmd_vel`: `ros2 topic echo /cmd_vel`
4. Tuning PID secara bertahap (mulai dari P gain)

### ❌ Build Error

```bash
# Clean build sepenuhnya
cd ~/ros2_ws
rm -rf build/ install/ log/

# Rebuild
colcon build --symlink-install

# Jika masih error, cek dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### ❌ Topic tidak muncul

```bash
# Cek apakah node berjalan
ros2 node list

# Cek remapping topic
ros2 node info /drone_controller

# Restart semua nodes
```
## 🤝 Kontribusi

Kontribusi sangat terbuka! Ikuti langkah berikut:

1. **Fork** repository ini
2. Buat **branch** fitur (`git checkout -b feature/FiturBaru`)
3. **Commit** perubahan (`git commit -m 'Menambahkan fitur baru'`)
4. **Push** ke branch (`git push origin feature/FiturBaru`)
5. Buat **Pull Request**

---



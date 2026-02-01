# hamals_firmware
Embedded firmware for Hamal’s mobile robot (motor control, encoders, IMU, safety logic).

# Ros-Mikrodenetleyici Köprüsü

```yaml
┌──────────────────────────────┐
│        ROS 2 (PC / SBC)       │
│                              │
│  LiDAR (USB)                  │
│  SLAM / Nav2 / EKF            │
│  diff_drive_controller        │
│                              │
└──────────────┬───────────────┘
               │  serial / usb
┌──────────────▼───────────────┐
│     Mikrodenetleyici          │
│                              │
│  Encoder (interrupt)          │
│  IMU (I2C / SPI)              │
│  Motor Driver (PWM / DIR)     │
│                              │
└──────────────────────────────┘
```

# Dosya Yapısı

```yaml
firmware/
├── main.ino                  ← SADECE setup() + loop() (orkestrasyon)
│
├── config/
│   ├── config_pins.h         ← PIN’ler, PWM, timer, board-specific
│   └── config_robot.h       ← wheel radius, track width, CPR, limitler
│
├── encoder/
│   ├── encoder.h
│   └── encoder.cpp           ← quadrature interrupt, pulse sayımı
│
├── imu/
│   ├── imu.h
│   └── imu.cpp               ← IMU raw okuma (fusion YOK)
│
├── motor/
│   ├── motor.h
│   └── motor.cpp             ← PWM
│
├── control/
│   ├── velocity_cmd.h
│   └── velocity_cmd.cpp      ← cmd_vel parse + hedef teker hızları
│   └── wheel_pid.cpp
│   └── wheel_pid.h
│
├── safety/
│   ├── watchdog.h
│   └── watchdog.cpp          ← cmd_vel timeout → FAIL-SAFE STOP
│
├── kinematics/
│   ├── kinematics.h
│   └── kinematics.cpp        ← pulse → rad / rad/s dönüşümü
│
├── timing/
│   ├── timing.h
│   └── timing.cpp            ← millis tabanlı scheduler (delay YOK)
│
├── comm/
│   ├── serial_comm.h
│   └── serial_comm.cpp       ← ROS ↔ MCU paket formatı
│
├── debug/
│   └── debug.h               ← debug flag’leri (compile-time)
│
└── README.md        ← firmware mimarisi + veri akışı
```

## Veri Akışı

```yaml
Encoder → MCU → JointState → diff_drive_controller → /odom_raw
IMU     → MCU → /imu       → EKF
LiDAR   → USB → /scan      → SLAM / Nav2
```


### Hertz Ayarları

```csharp
loop():
  comm.receive()
  timing.tick()

  if (1ms)   encoder.snapshot()
  if (10ms)  kinematics.update()
  if (10ms)  wheel_pid.update()
  if (100ms) safety.check()
  if (50ms)  comm.sendFeedback()
  
   MCU loop hedefi: ≥ 1 kHz
```

```csharp
MCU PID            : 100 Hz
MCU Feedback       : 50 Hz
JointState         : 50 Hz
IMU                : 50–100 Hz
EKF                : 30–50 Hz
TF                 : 50 Hz
Nav2 Controller    : 20 Hz
Nav2 Planner       : 1 Hz
Costmap update     : 5 Hz
Watchdog timeout   : 200–300 ms
```
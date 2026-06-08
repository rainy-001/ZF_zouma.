# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

21届智能车走马观碑组 — an embedded smart car racing project targeting the **LS2K0300 (LoongArch64)** SoC. The codebase is C++17/C99, built on 逐飞科技 (SeekFree) open-source底层 libraries, and cross-compiled from x86_64 Linux.

## Build & Deploy

```bash
# Full build+deploy pipeline: clean → cmake → make → SCP to target
cd project/user && bash build_test.sh

# Manual steps (from project/out/):
cd project/out
cmake ../user                          # configure (uses cross.cmake for LoongArch64)
make -j$(nproc)                        # compile
scp -O project root@192.168.79.125:/home/root/  # deploy to car
```

**Cross-compilation toolchain:** `/opt/ls_2k0300_env/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6/bin/loongarch64-linux-gnu-g++`
**OpenCV:** `/opt/ls_2k0300_env/opencv_4_10_build`
**NCNN:** statically linked from `libraries/zf_components/ncnn/lib/libncnn.a`

The build output goes to `project/out/project` (the `project/` directory name becomes the executable name).

## Architecture

### Layer Stack (top to bottom)

```
project/user/main.cpp          ← entry point: car_init() → main loop
project/code/                  ← application logic
libraries/zf_components/       ← SeekFree assistant, NCNN
libraries/zf_device/           ← device drivers (IPS200 display, IMU, UVC camera)
libraries/zf_driver/           ← hardware abstraction (PWM, GPIO, Encoder, PIT timers, TCP/UDP)
```

All headers are aggregated through `libraries/zf_common/zf_common_headfile.hpp` — new application files must be registered there.

### Thread Model (LS2K0300 PIT hardware timers)

Three real-time threads, priorities in descending order:

| Thread | Priority | Period | Handler |
|--------|----------|--------|---------|
| Encoder speed sampling | 99 | ENCODER_SAMPLING_PERIOD (2ms) | `encoder_get_count_handler()` |
| LADRC/PID control | 97/99 | 1ms / PID_CONTROL_PERIOD (5ms) | `hight_frequence_encoder_get_speed_handler()` or `pid_contol_handle()` |
| Key scan + IMU + AHRS | 95 | KEY_SCAN_PERIOD (10ms) | `key_scan_handler()` |

Thread callbacks are declared in `my_global.hpp` and implemented in `my_global.cpp`. `car_init()` reads `control_model` from config to decide PID vs LADRC, then starts the appropriate threads.

### Control Algorithms

- **`MyPID`** (`my_pid.hpp`): Full-featured PID with anti-windup, derivative low-pass filter, feedforward, reverse action. Used for wheel speed control.
- **`PDController`** (`my_pid.hpp`): Nonlinear PD for steering — uses both linear Kp and square-term Kp2. Computes `onto_control` (steering correction).
- **`LADRC`** (`LADRC.hpp`): Linear Active Disturbance Rejection Control with Tracking Differentiator (TD) → Extended State Observer (ESO) → Linear State Error Feedback (LSEF). Wrapped by `SimpleMotorLADRC` for PWM output.
- **`calculate_yaw_control()`** (`IMU963R.hpp`): Converts yaw error to bounded steering control signal.

### Configuration System

`car_config.txt` on the target (`/home/root/car_config.txt`) is parsed at boot. It selects control mode and sets all PID/LADRC parameters. The parsing logic is in `config_setting.cpp`/`config_seting.hpp`. Parameters are `extern` globals defined in `config_setting.cpp`.

### vofa+ Telemetry & Tuning

The main loop (`main.cpp`) runs a TCP client that:
1. Listens for commands (`READ`, `WRITE`, or parameter-update strings)
2. Sends telemetry frames every 10ms via `sendFormattedData()`

vofa+ configuration files are at the repo root: `vofa+.cmds.json` and `vofa_adjust_ui.cmds.json`.

### Menu System

Button-driven hierarchical menu rendered on IPS200 display. Menu entries are defined as a static array `Menu MyMenu::menu_table[]` in `my_menu.cpp`. Each entry has an ID, parent ID, name, and handler callback. Handler wrappers with key mapping are implemented in `my_task_function.cpp`.

### Image Processing Pipeline

(`imgproc.hpp`/`imgproc.cpp`)

1. `image_proc()` — main processing entry
2. `line_process()` — extracts left/right boundaries, corner points, max curvature
3. `element_status()` — state machine determining current track element (straight, crossing, circle)
4. Per-state handlers: `no_element_process()`, `crossing_process()`, `circle_process()`, `auto_tracking()`

**Known issue:** `calculate_weighted_offset_angle` has a regression bug from a version rollback — it is unstable and should not be used. Modify state machine logic first, not the low-level algorithms.

### Inertial Navigation

(`navigation.hpp`/`navigation.cpp` + `akima.hpp`)

- **`PathTracker`**: Records path points (x, y, yaw) using dual-wheel odometry (`Odometer`), supports Gaussian smoothing. Can save/load binary maps.
- **`AkimaInterpolator`**: Akima spline interpolation for map data. Map format: index x y in text files.
- Recording is toggled through menu system; reproduction uses closest-index search with forward window.

### IMU & AHRS

- **`IMUHandler`** (`IMU963R.hpp`): Reads raw IMU via Linux IIO, applies low-pass filtering and zero-offset calibration.
- **`MadgwickAHRS`**: Attitude heading reference system — computes yaw/pitch/roll from gyro + accelerometer.

### NCNN Classification

(`tflm_model_process_lq.hpp`): NCNN-based TinyClassifier for image element classification (40×40 input). Model loaded from `.param` and `.bin` files on the target. Currently disabled in `car_init()`.

## Key Files Reference

| File | Role |
|------|------|
| `project/user/main.cpp` | Entry point, main loop with TCP telemetry |
| `project/user/CMakeLists.txt` | Build config, TTF option, NCNN linking |
| `project/user/cross.cmake` | LoongArch64 cross-compilation toolchain |
| `project/code/my_global.hpp` | Global objects, externs, thread periods, callback declarations |
| `project/code/my_global.cpp` | `car_init()` — full hardware init sequence; thread callback implementations |
| `project/code/my_pid.hpp` | MyPID, PDController, SimpleMotorLADRC classes |
| `project/code/LADRC.hpp` | Core LADRC algorithm (TD + ESO + LSEF) |
| `project/code/motor.hpp` | PWM drive + encoder speed mapping + LADRC speed getter |
| `project/code/config_seting.hpp` | TCPClient class, config file parsing, all parameter globals |
| `project/code/my_menu.hpp` | Menu struct, MyMenu class with menu_table |
| `project/code/my_task_function.cpp` | Menu handler wrappers with key mapping |
| `project/code/imgproc.hpp` | Tracking decision machine state machine, image processing declarations |
| `project/code/IMU963R.hpp` | IMU handler with filtering and yaw control calculation |
| `project/code/navigation.hpp` | Odometer, PathTracker, map loading/reproduction |
| `libraries/zf_common/zf_common_headfile.hpp` | Master include — register new files here |

## TTF Font System

Controlled by CMake option `USE_TTF` (ON by default) in `project/user/CMakeLists.txt`. When enabled:
- `xxd -i` converts `assets/fonts/MiSans_Mini.ttf` into `font_data.cpp` at build time
- Defines `WHETHER_USE_TTF=1` preprocessor macro
- The font covers 7000+ Chinese characters plus all Latin characters

## Target Device

- IP: `192.168.79.125` (wired) / `192.168.79.38` (TCP host)
- User: `root`
- Config file: `/home/root/car_config.txt`
- Model files (when enabled): `/home/root/models/`

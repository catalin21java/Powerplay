# FTC Power Play — DecebalTech

Robot control software for **DecebalTech** during the **2022–2023 FIRST Tech Challenge season**, [*Power Play*](https://www.firstinspires.org/robotics/ftc/game). This repository contains the team's Java-based autonomous and teleop code, built on the FTC SDK and [Road Runner](https://github.com/acmerobotics/road-runner).

## Overview

The codebase drives a mecanum-chassis robot with an articulated arm, dual-slider lift, and servo gripper. Autonomous routines use Road Runner trajectory following with AprilTag vision to read the signal zone, while teleop modes provide manual, semi-automatic, and stack-scoring controls for cone placement on junction poles.

## Features

- **Mecanum drive** with encoder-based odometry and BNO055 IMU heading
- **Road Runner** trajectory sequences for precise autonomous navigation
- **AprilTag vision** via EasyOpenCV for signal-zone detection and pose correction
- **Articulated manipulator** — dual synchronized sliders, rotating arm, and servo gripper
- **Color sensing** for cone detection during intake
- **FTC Dashboard** integration for live tuning of drive and mechanism constants
- **PhotonFTC** for improved control-loop performance on the REV Control Hub
- **MeepMeep** module for offline autonomous path visualization

## Tech Stack

| Component | Version / Library |
|-----------|-------------------|
| Language | Java |
| Build | Gradle, Android Studio |
| FTC SDK | 8.0.0 |
| Drive framework | [Road Runner](https://github.com/acmerobotics/road-runner) |
| Dashboard | [FTC Dashboard](https://github.com/acmerobotics/dashboard) 0.4.5 |
| Vision | EasyOpenCV, OpenFTC AprilTag |
| Performance | [PhotonFTC](https://github.com/Eeshwar-Krishnan/PhotonFTC) v2.0.1 |
| Game assets | `gameAssets-PowerPlay` 1.0.0 |

## Project Structure

```
ftc_powerplay/
└── main-code/Poweplay-main/
    ├── TeamCode/              # Team robot code (op modes, drive, utilities)
    │   └── src/main/java/org/firstinspires/ftc/teamcode/
    │       ├── autonomus/     # Autonomous routines (blue/red variants)
    │       ├── drive/         # Road Runner drive, tuning, and teleop modes
    │       ├── samplePID/     # PID tuning and motor control experiments
    │       ├── trajectorysequence/  # Multi-segment trajectory support
    │       └── util/          # RobotUtils, dashboard helpers, encoders
    ├── FtcRobotController/    # FTC Robot Controller app (SDK)
    ├── MeepMeepTesting/       # MeepMeep path preview
    ├── libs/                  # Local JAR/AAR dependencies
    └── build.dependencies.gradle
```

## Getting Started

### Prerequisites

- [Android Studio](https://developer.android.com/studio) (compatible with Gradle Plugin 4.2.x)
- A REV Control Hub or Expansion Hub with the FTC Robot Controller app
- A Driver Station device running the FTC Driver Station app
- USB connection or wireless debugging access to the robot

### Setup

1. **Clone the repository**

   ```bash
   git clone https://github.com/catalin21java/ftc_powerplay.git
   cd ftc_powerplay/main-code/Poweplay-main
   ```

2. **Open in Android Studio**

   Open the `Poweplay-main` directory as a project. Android Studio will sync Gradle dependencies automatically.

3. **Build the project**

   Select the `TeamCode` module and build, or run from the terminal:

   ```bash
   ./gradlew :TeamCode:assembleDebug
   ```

4. **Deploy to the robot**

   Connect to the Control Hub, then use Android Studio's **Run** button or `./gradlew :TeamCode:installDebug` to install the app on the robot phone.

> **Note:** If you encounter multidex build issues, set `useProguard` to `true` in `build.common.gradle`.

## Op Modes

### TeleOp

| Op Mode | Description |
|---------|-------------|
| `DRIVE` | Primary driver mode with auto/manual/stack control modes and pole-height selection |
| `DRIVE_NORMAL` | Simplified teleop variant |
| `TeleOpDrive` | Road Runner–based field-centric driving |
| `TeleOpFieldCentric` | Field-centric drive with IMU heading |

### Autonomous

Autonomous routines are organized by alliance color and starting position:

| Op Mode | Alliance | Starting position |
|---------|----------|-------------------|
| `Auto_Left_Mid_Blue` | Blue | Left / mid |
| `Auto_RIGHT_Mid_Blue` | Blue | Right / mid |
| `Auto_Left_Mid_Red` | Red | Left / mid |
| `Auto_RIGHT_Mid_Red` | Red | Right / mid |
| `Auto_RIGHT_Blue` / `Auto_RIGHT_Red` | Blue / Red | Right-side with compensation |
| `Auto_reposition` | — | Repositioning routine |

Additional variants under the `COMPENSARE` group provide adjusted trajectories for specific field conditions.

### Drive Tuning

Road Runner tuning op modes live under the `drive` group (e.g. `TrackWidthTuner`, `DriveVelocityPIDTuner`, `AutomaticFeedforwardTuner`). Run these on the field to calibrate constants stored in `DriveConstants.java`.

## Configuration

Key files to review before running on hardware:

| File | Purpose |
|------|---------|
| `drive/DriveConstants.java` | Track width, feedforward, velocity/acceleration limits |
| `util/RobotUtils.java` | Arm/slider positions, servo limits, sensor thresholds |
| `autonomus/*.java` | Starting poses, trajectory waypoints, AprilTag camera calibration |

Connect to the robot's WiFi network and open **FTC Dashboard** at `https://192.168.49.1:8080/dash` to tune `@Config` values at runtime. Save tuned values back to source files — dashboard changes do not persist across app restarts.

## Hardware Map

Expected device names configured in the Robot Configuration:

| Name | Type | Role |
|------|------|------|
| `slider1`, `slider2` | DcMotor | Dual synchronized lift sliders |
| `rotire` | DcMotor | Arm rotation |
| `gheara` | Servo | Gripper |
| `color` | RevColorSensorV3 | Cone detection |
| `dist` | RevColorSensorV3 | Distance sensing |
| `Webcam 1` | Webcam | AprilTag vision |

Drive motors follow the standard Road Runner mecanum naming convention configured in `SampleMecanumDrive.java`.

## Acknowledgments

This project builds on open-source tools and libraries maintained by the FTC community:

- [Road Runner](https://github.com/acmerobotics/road-runner) — Acme Robotics
- [FTC Dashboard](https://github.com/acmerobotics/dashboard) — Acme Robotics
- [EasyOpenCV](https://github.com/OpenFTC/EasyOpenCV) — OpenFTC
- [PhotonFTC](https://github.com/Eeshwar-Krishnan/PhotonFTC) — Eeshwar Krishnan
- [FIRST Tech Challenge SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController)

## License

Team-specific code in `TeamCode/` is provided for reference and learning. Third-party libraries and the FTC SDK retain their respective licenses. See `FtcRobotController/LICENSE` and `doc/legal/` for SDK and asset license details.

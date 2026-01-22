# YILDIZ USV - System Architecture & Autonomy Guide

## 1. System Overview
The **YILDIZ USV** uses a high-speed, reactive autonomy stack designed for parkour navigation. Unlike traditional map-based systems, it processes LiDAR data in real-time to make split-second decisions, ensuring robust obstacle avoidance and channel navigation.

## 2. Data Flow Architecture

```mermaid
graph TD
    %% Styling
    classDef sensor fill:#2a2a2a,stroke:#00ff99,stroke-width:2px,color:#fff;
    classDef process fill:#1a1a1a,stroke:#00aaff,stroke-width:2px,color:#fff;
    classDef decision fill:#330033,stroke:#ff00ff,stroke-width:2px,color:#fff;
    classDef actuator fill:#440000,stroke:#ff3333,stroke-width:2px,color:#fff;

    subgraph Hardware [Sensors & Hardware]
        Lidar[Velodyne VLP-16]:::sensor -->|PointCloud2| PCL_Filter[LiDAR Filter Node]:::process
        IMU[Xsens IMU]:::sensor -->|Data| EKF[State Estimation]:::process
        GPS[Garmin GPS]:::sensor -->|Fix| EKF
    end

    subgraph Perception_Pipeline [Perception Pipeline]
        PCL_Filter -->|Filtered Points| PC2Laser[PointCloud -> LaserScan]:::process
        PC2Laser -->|Scan Ranges| FGM_Node[Reactive Obstacle Avoidance]:::process
    end

    subgraph FGM_Logic [FGM Algorithm Logic]
        FGM_Node --> Preprocess{Preprocess Scan}:::decision
        Preprocess -->|Limit Checking| FOV[FOV Clipping\n(-90 to +90 deg)]:::process
        FOV -->|Clean Data| Inf_Check[Replace Inf/NaN]:::process
        Inf_Check -->|Ranges| Bubble[Safety Bubble\nTarget: Closest Point]:::process
        Bubble -->|Masked Array| Find_Gaps[Find Contiguous Gaps]:::process
        Find_Gaps -->|Gap List| Score_Gaps{Score Gaps}:::decision
        Score_Gaps -->|Center Bias| Formula["Score = Len / (1 + 3*|Ang|)"]:::process
        Formula -->|Best Gap| Target[Select Target Angle]:::process
    end

    subgraph Control_System [Control & Actuation]
        Target -->|Error| PID[P-Controller]:::process
        PID -->|Twist API| Cmd_Vel[/cmd_vel]:::process
        Cmd_Vel -->|Linear/Angular| Converter[Thruster Converter]:::process
        Converter -->|Differential Drive| Left_Motor[Left Thruster]:::actuator
        Converter -->|Differential Drive| Right_Motor[Right Thruster]:::actuator
    end

    %% Data Connections
    EKF -.->|Odom| FGM_Node
```

## 3. Core Algorithms

### A. Follow the Gap Method (FGM)
We utilize a modified **Follow the Gap** algorithm, widely used in F1/10 autonomous racing.
1.  **Safety Bubble**: Inflates obstacles virtually to prevent clipping corners.
2.  **Max Gap Detection**: Identifies the largest navigable sector in the LiDAR scan.
3.  **Target Selection**: Steers towards the optimal point in that gap.

### B. FOV Clipping (Backtracking Prevention)
To solve the "U-Turn" problem where the robot sees open space behind it and tries to turn back:
-   **Logic:** The LiDAR scan is clipped to only consider the **Front 180° (-90° to +90°)**.
-   **Result:** The robot is "blind" to gaps behind it, forcing forward progress.

### C. Center Bias (Parkour Entry)
Standard FGM picks the *widest* gap, which often leads to the open sea instead of a narrow parkour channel.
-   **Logic:** We apply a scoring function to all gaps:
    $$ Score = \frac{Length}{1.0 + 3.0 \times |Angle|} $$
-   **Result:** A gap directly ahead (0°) gets a 3x multiplier bonus compared to a gap at the side (90°). This actively prioritizes entering the channel.

## 4. Motor Control & Optimization
The `converter` node translates standard ROS navigation commands (`Twist`) into differential thrust.

-   **Linear Scale:** `5.0` (Aggressive forward thrust)
-   **Angular Scale:** `25.0` (Sharp turning capability)
-   **Steering Gain (Kp):** `2.0` (Fast reaction time)

## 5. Usage
To launch the full parkour autonomy stack:

```bash
./start_all.sh parkour
```

This script handles:
1.  Process cleanup (killing old nodes).
2.  Gazebo Simulation launch.
3.  Sensor drivers & filters.
4.  **Reactive FGM Node** (instead of Nav2).
5.  Thruster Converter.

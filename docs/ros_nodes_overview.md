# ROS Node Architecture Overview

This document summarizes the ROS nodes included in the Spot Micro workspace, how they exchange data, and highlights opportunities to simplify the stack. Nodes are grouped by subsystem to clarify their responsibilities and communication paths.

## Motion Control Core

### `spot_micro_motion_cmd`
*Language: C++*

The primary control node manages robot state via a finite state machine with idle, stand, walk, and transition states. It consumes discrete mode events and continuous body commands to produce servo commands and auxiliary telemetry.

**Subscriptions**
- `/stand_cmd`, `/idle_cmd`, `/walk_cmd` (`std_msgs/Bool`): state-change events issued by teleoperation nodes.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L76-L89】
- `/angle_cmd` (`geometry_msgs/Vector3`): desired body roll/pitch/yaw angles used in stand mode.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L84-L85】【F:spot_micro_motion_cmd/src/smfsm/spot_micro_stand.cpp†L33-L61】
- `/cmd_vel` (`geometry_msgs/Twist`): desired planar velocities applied in walk mode.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L87-L88】【F:spot_micro_motion_cmd/src/smfsm/spot_micro_walk.cpp†L41-L55】

**Publications & Services**
- `servos_proportional` (`i2cpwm_board/ServoArray`): continuous joint commands computed from inverse kinematics and normalized to ±1.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L93-L95】【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L243-L275】
- `servos_absolute` (`i2cpwm_board/ServoArray`): zero commands for releasing servos when idling.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L90-L92】【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L278-L280】
- `config_servos` service client (`i2cpwm_board/ServosConfig`): pushes calibration parameters to the servo driver at startup.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L96-L98】【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L183-L214】
- `body_state` (`std_msgs/Float32MultiArray`): 18-element telemetry vector used for visualization and debugging.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L99-L100】【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L485-L518】
- `lcd_state`, `lcd_vel_cmd`, `lcd_angle_cmd`: status mirrors of the FSM, velocity, and attitude commands originally intended for an LCD monitor.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L102-L109】【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L522-L535】
- TF transforms for links, feet, and optional odometry frames, published every cycle.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L138-L176】【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L539-L609】

The node houses the gait controller (`SpotMicroWalkState`) that advances swing and stance phases and integrates velocity commands, as well as stand/idle modes that filter angle commands before issuing servo updates.【F:spot_micro_motion_cmd/src/smfsm/spot_micro_walk.cpp†L41-L185】【F:spot_micro_motion_cmd/src/smfsm/spot_micro_stand.cpp†L33-L63】

### Teleoperation Nodes
*Language: Python*

Two nodes provide operator input and share the same command topics consumed by `spot_micro_motion_cmd`.

- `spot_micro_keyboard_control`: Presents a CLI menu, publishing motion events and body commands on `/stand_cmd`, `/idle_cmd`, `/walk_cmd`, `/angle_cmd`, and `/cmd_vel`. It also zeroes commands when switching modes.【F:spot_micro_keyboard_command/scripts/spotMicroKeyboardMove.py†L81-L115】【F:spot_micro_keyboard_command/scripts/spotMicroKeyboardMove.py†L141-L199】
- `spot_micro_joystick_control`: Maps joystick buttons and axes to the same topics, switching between idle, stand, angle, and walk modes based on button presses.【F:spot_micro_joy/scripts/spotMicroJoystickMove.py†L68-L154】

Because both nodes duplicate message definitions and state handling logic, they are strong candidates for consolidation or for adoption of standard teleop packages (e.g., `teleop_twist_keyboard` plus `joy_teleop`).

### Actuator Interface

The `i2cpwm_board` package (pulled as a submodule) hosts the servo driver node referenced by the launch files. `spot_micro_motion_cmd` assumes exclusive ownership of `servos_proportional` while occasionally commanding `servos_absolute` during idle transitions. Calibration and service calls occur during startup through `publishServoConfiguration()`.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L183-L214】

Calibration helpers include:
- `servo_move_keyboard`: publishes raw `servos_absolute` values to position or calibrate joints manually.【F:servo_move_keyboard/scripts/servoMoveKeyboard.py†L177-L194】
- `servo_walk`: generates stepping trajectories by reusing the inverse kinematics engine and streaming joint values to `servos_absolute`, independent of the main FSM.【F:servo_walk/scripts/servo_walk.py†L119-L200】【F:servo_walk/scripts/servo_walk.py†L247-L286】

## Visualization and Simulation

- `spot_micro_plot`: A matplotlib-based stick-figure viewer that blocks waiting on `body_state` telemetry and renders the robot in 3D. It is primarily useful when `spot_micro_motion_cmd` runs in standalone debug mode.【F:spot_micro_plot/scripts/spotMicroPlot.py†L47-L170】
- `spot_micro_rviz`: Launch files (`show_model.launch`, `slam.launch`) start RViz with the URDF model and optional SLAM overlays. These launchers rely on TF published by `spot_micro_motion_cmd` and the `rplidar_ros` driver when mapping.【F:spot_micro_launch/launch/keyboard_control_and_rviz.launch†L9-L19】【F:spot_micro_launch/launch/motion_control_and_hector_slam.launch†L8-L48】

## Peripheral UI and Monitoring

### OLED Display Stack

- `display_manager`: Aggregates system telemetry (CPU load, memory, disk, temperature) via shell commands and publishes formatted text to `oled_text`. It also manages background color through `oled_color` based on button presses and ultrasonic readings.【F:display_manager/scripts/display_manager.py†L18-L145】
- `oled_display`: Runs on the embedded controller, subscribing to `oled_color` and `oled_text` to redraw the SSD1331 OLED panel.【F:oled_display/scripts/oled_display.py†L40-L75】
- `button_monitor`: Monitors a physical button (GPIO 22) and publishes `button_press` events, which the display manager uses to cycle through modes.【F:button_monitor/scripts/button_monitor.py†L10-L33】
- `ultrasonic_monitor`: Streams left/right distance measurements from a serial ultrasonic sensor onto `ultrasonic_data`, allowing the display manager to flag close obstacles in its alert mode.【F:ultrasonic_monitor/scripts/ultrasonic_monitor.py†L7-L44】

### System Service Status

`system_status` polls `systemctl` for the `motion`, `display`, and `rosbridge` services and publishes integer flags on `/motion_status`, `/display_status`, and `/rosbridge_status`. No other package in this repository subscribes to those topics, so the node currently has no in-tree consumers.【F:system_status/scripts/system_status.py†L8-L41】【c139a7†L1-L5】【ca8d2b†L1-L5】

## Launch Coordination

The `spot_micro_launch` package bundles launch files that compose the above nodes for common workflows. `keyboard_control_and_rviz.launch` starts the keyboard teleop node plus either the pure visualization or SLAM RViz configuration. `motion_control_and_hector_slam.launch` adds the motion controller, optional `i2cpwm_board`, the RPLidar driver, and Hector SLAM for mapping sessions.【F:spot_micro_launch/launch/keyboard_control_and_rviz.launch†L5-L19】【F:spot_micro_launch/launch/motion_control_and_hector_slam.launch†L8-L48】

## Opportunities to Simplify

1. **Unify teleop inputs.** The keyboard and joystick nodes each reimplement command-state tracking and topic publishing. Refactoring them to share a common helper library or delegating to standard teleop packages would reduce maintenance overhead.【F:spot_micro_keyboard_command/scripts/spotMicroKeyboardMove.py†L81-L199】【F:spot_micro_joy/scripts/spotMicroJoystickMove.py†L68-L154】
2. **Streamline display status reporting.** `system_status` publishes service-health integers that no other node uses, while `display_manager` already gathers system metrics asynchronously. Either integrate service checks into `display_manager` or remove `system_status` to avoid an idle publisher.【F:system_status/scripts/system_status.py†L21-L39】【c139a7†L1-L5】
3. **Evaluate legacy servo utilities.** Both `servo_move_keyboard` and `servo_walk` talk directly to `servos_absolute`, bypassing the FSM. If calibration is infrequent, these scripts could be consolidated or converted into roslaunch-able tests to avoid multiple pathways commanding the same servos.【F:servo_move_keyboard/scripts/servoMoveKeyboard.py†L171-L206】【F:servo_walk/scripts/servo_walk.py†L167-L214】
4. **Retire unused LCD topics.** `spot_micro_motion_cmd` still advertises `lcd_state`, `lcd_vel_cmd`, and `lcd_angle_cmd`, but no subscriber exists in this repository.【F:spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp†L102-L109】【4c2f00†L1-L9】 Removing them (or wiring them into the OLED display stack) would simplify the data model.
5. **Consider collapsing display stack responsibilities.** `display_manager` clears and repopulates the OLED every message, while `oled_display` rebuilds the entire scene on each callback.【F:oled_display/scripts/oled_display.py†L40-L75】 Folding the color/text update logic into a single node could reduce redraw churn and eliminate the custom inter-node protocol if embedded constraints allow.

These changes would trim overlapping functionality and reduce the number of custom topics that need to be coordinated, making the system easier to maintain.

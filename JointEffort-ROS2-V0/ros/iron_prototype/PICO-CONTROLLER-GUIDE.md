# Controller input

The Pico reads the controls and reports them as a USB joystick. ROS turns those reports into tip movement requests. This workflow moves only the mock arm.

## Simulated controls

Start the model in one terminal:

```bash
./scripts/run_iron.sh
```

From `ros2_ws`, start input processing in a second terminal:

```bash
./scripts/run_input.sh joystick:=false mapping_confirmed:=true
```

Run the simulated controls in a third terminal:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/input_demo.py
```

The demo moves up and down, changes mode, and changes the tool's pointing direction. Do not run a real joystick reader at the same time.

## Real controller check

Connect the Pico to Ubuntu. For a VM, assign the USB device to the Ubuntu guest.

```bash
./scripts/with_ros.sh ros2 run joy joy_enumerate_devices
```

The observed name is `Joint Effort AMW Controller`. Start its reader:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh ros2 run joy joy_node --ros-args -p device_name:="Joint Effort AMW Controller" -p deadzone:=0.0 -p autorepeat_rate:=50.0 -p sticky_buttons:=false
```

In another terminal:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/input_probe.py --seconds 60
```

Move one axis at a time. Press and hold each button separately. Turn the dial to both ends. The probe saves the values to `reports/iron/input-mapping-observed.json` and sends no movement commands.

Check the observed indexes and directions against `src/joint_effort_input/config/input.yaml`. Stop the separate reader before using `run_input.sh`. Set `mapping_confirmed:=true` only after reviewing the mapping.

## Current ROS mapping

| Control | Setting |
| --- | --- |
| Stick axes | 0, 1, 2; provisional positive signs |
| Encoder speed field | Axis 3 |
| Enable | Hold raw button 0 |
| Mode | Raw button 1 switches translation/rotation |
| Translation | X/Y/twist request world X/Y/Z motion |
| Rotation | X/Y request two pointing components; twist is unused |
| Deadzone | 8% at the stick center |
| Speed | Levels 1/10/20 give 0.1x/1x/2x |

Center the stick and release enable for at least 0.2 seconds. Press enable while centered, then move. Release to stop. Mode changes and faults require the same center/release step.

The old Pico program reads analog inputs GP26/27/28, buttons GP2 through GP7, and encoder GP10/11/12. The team's JackTestV1 program uses encoder GP18/19/20 and toggles enable in firmware. Raw readings from the prototype changed on all three analog inputs and on the encoder; that does not confirm the assembled panel's full mapping.

Our ROS code owns mode and enable. Reconcile the Pico's toggle flags and display behavior before using JackTestV1 for normal control. The proposed four-button panel and physical slider remain unconfigured.

## Input checks

The forwarder checks messages, fresh feedback, active mock controllers, collision status, and command ownership. Bad or stale input stops requests. A held button cannot restart motion after a disconnect.

Host-side joystick repetition does not prove the Pico firmware is healthy. A separate firmware heartbeat, motor watchdog, and physical stop are still needed before hardware operation.

The terminal preview shows mode, speed setting, and faults. The physical display connection still needs to be confirmed.

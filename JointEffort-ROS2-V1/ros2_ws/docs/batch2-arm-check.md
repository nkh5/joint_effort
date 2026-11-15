# Batch 2 arm check (about 30 min)

Batch 2 changes how the arm is driven, not where it is: one `amw_teleop` node instead of
forwarder + bridge + joy_teleop, the ON/OFF latch on the Pi, one status with stop reasons, limit
and singularity escape, and the LCD. Do docs/batch1-arm-check.md steps 0-5 first (same angles,
directions, hold, jogs), then this.

Safety as always: 12 V disconnect within reach, support the arm whenever torque goes off.

## 0. Code on the Pi

`WorkingV1` has everything. On the Pi:

```bash
cd ~/joint_effort && git pull
cd JointEffort-ROS2-V1/ros2_ws
amw build --clean --workers 1
amw test
```

Check: colcon `Summary: 6 packages finished` (`joy_teleop` is gone); `amw test` ends with `OK`.

## 1. Pico firmware (laptop)

Arduino IDE: board "Raspberry Pi Pico" (earlephilhower core), Tools > USB Stack > "Adafruit
TinyUSB". Open `Pico_Controller/Pico_Controller.ino`, upload.

Plug the Pico into the Pi, start the arm (`amw start real` or `amw start mock`), and in a second
terminal:

```bash
amw joystick
```

Check: pressing the stick button shows `pressed [0]`, panel 1 `pressed [1]`, panel 2 `pressed [2]`
... encoder push `pressed [6]`. Nothing ever shows 7 or 8 (those were the old ON bits).

The new Pi software also works with the old firmware (it only reads the raw buttons 0 and 1), so the
order does not matter.

Note down: buttons OK.

## 2. LCD

Follow docs/lcd.md "Pi setup" (overlay, `python3-pygame`, service). Then:

Check: with `amw start` stopped the LCD shows NO DATA; after `amw start real` it shows OFF.

Note down: LCD OK.

## 3. Driving (Pi)

Arm inside its limits, nothing near the iron.

```bash
amw start real
```

Second terminal:

```bash
amw status
```

Speed level 5-8. Do each row, watch the LCD, `amw status` and the `amw start` terminal:

| do this | expected (LCD / status / terminal) |
| --- | --- |
| nothing (just started) | OFF; `[OFF] Arm is off.` |
| panel 1, stick centred | CENTRE STICK for 0.2 s, then ON (green); `[ON] moving` |
| stick +X / +Y / twist + | tip moves forward / left / up |
| panel 1 | OFF; `[OFF] Turned off with panel 1.` |
| hold the stick over, press panel 1 | CENTRE STICK until you let go, then ON |
| stick button | CENTRE STICK "Switched to rotation mode."; let go -> ON in rotation mode |
| rotation, stick X / Y | iron yaw / pitch, tip stays put |
| stick button again, centre | back to translation |
| drive one joint to its limit (e.g. keep lowering until the elbow limit) | stays **ON**, amber strip "joint_N is at its ... limit"; the stick the other way moves away. (Before: the whole arm froze.) |
| `amw stop` in a third terminal | STOPPED "Stopped by `amw stop`."; panel 1 turns it back ON |
| unplug the Pico USB while ON | NO CONTROLLER; terminal `[STOP] No joystick input ...` |
| plug it back in | stays OFF until panel 1 is pressed |
| Ctrl-C `amw start` | LCD: NO DATA |

`amw status` should show every one of these in its "Recent" list with how long ago it happened.

Optional, carefully: drive towards a stretched-out arm. Expected: an amber SINGULARITY strip and
slower motion in the direction it cannot follow, not a freeze.

Note down: each row OK / not, and anything the screens said that was unclear.

## Report back

Send the notes. Anything that stopped without a clear reason on the LCD or in `amw status` is a bug
in the fault table: tell me what you did and what it said.

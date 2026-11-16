# Joy-Con controller

A left Joy-Con, held upright in one hand, replaces the Pico joystick.

| Input | Does |
| --- | --- |
| stick | tip x/y (rate: further = faster; both directions reversed by `stick_signs: [-1, -1]`) |
| hold **L** + stick up/down | tip down/up (half the x/y speed; `z_sign: -1` = stick up moves down) |
| hold **ZL** + point the Joy-Con | the iron points the way the Joy-Con points: Joy-Con upright (long side vertical, stick facing you) = iron straight down; tilt it and the iron tilts the same way. Release ZL to freeze the iron. Stick ignored. |
| D-pad left | "this is upright": hold the Joy-Con upright and press it to set the neutral pose (any time) |
| D-pad right | ON / OFF |
| D-pad up / down | speed level +2 / -2 (1..20, 10 = 1x) |

All numbers (deadzone, curve, speeds, which buttons, hand scale and directions) are in
`src/amw_bringup/config/teleop.yaml`; edit and restart `amw start`, no rebuild.

How it works: `amw_joycon` (src/amw_teleop/amw_teleop/joycon.py) reads the two input devices the
kernel's hid-nintendo driver creates, `Joy-Con (L)` and `Joy-Con (L) (IMU)`, and publishes `/joy`
(stick + buttons, 100 Hz) and `/amw/hand_imu` (gyro with its bias removed, ~200 Hz, stamped with the
Joy-Con's own sample clock). `amw_teleop` turns the stick into tip velocity. It also tracks the
Joy-Con's orientation all the time (gyro, with gravity correcting the tilt drift); while ZL is held it
swings the iron toward the direction the Joy-Con's long side points, relative to the neutral pose set
with D-pad left (hand.py). Without a neutral, the first ZL press uses the pose at that moment.
At start, `amw_joycon` measures the gyro bias and stick centre: **leave the Joy-Con still for a
second** (e.g. on the table) before it says `Ready.`

## One-time setup (Fedora host)

1. Pair the Joy-Con with an agent, so it bonds (our left Joy-Con is `7C:BB:8A:A5:57:A5`).
   Hold its small sync button (rail side) until the LEDs sweep, then one line at a time:

   ```bash
   bluetoothctl
   agent on
   default-agent
   scan on
   pair 7C:BB:8A:A5:57:A5
   trust 7C:BB:8A:A5:57:A5
   connect 7C:BB:8A:A5:57:A5
   scan off
   quit
   ```

   Check: `bluetoothctl info 7C:BB:8A:A5:57:A5 | grep -E "Bonded|Connected"` shows `yes` twice. Later,
   pressing any Joy-Con button reconnects it.

2. Stop `joycond`. It grabs the Joy-Con's input devices, and then nothing else can read them:

   ```bash
   sudo systemctl disable --now joycond
   ```

3. Let your user read the Joy-Con input devices (distrobox shares `/dev/input` with the host):

   ```bash
   sudo tee /etc/udev/rules.d/99-amw-joycon.rules >/dev/null <<'EOF'
   SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*Joy-Con*", MODE="0666"
   SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*Pro Controller*", MODE="0666"
   EOF
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

   Then disconnect and reconnect the Joy-Con (`bluetoothctl disconnect 7C:BB:8A:A5:57:A5`, press a button).
   Check, which lists the Joy-Con's two devices and their permissions (want `crw-rw-rw-`):

   ```bash
   for d in /dev/input/event*; do n=$(cat /sys/class/input/${d##*/}/device/name); \
     case "$n" in *Joy-Con*) ls -l "$d"; echo "  $n";; esac; done
   ```

4. Inside the ROS environment (the `ubuntu24` distrobox, or the Pi):

   ```bash
   sudo apt install python3-evdev
   python3 -c "import evdev; print([evdev.InputDevice(p).name for p in evdev.list_devices()])"
   ```

   Check: the list contains `Joy-Con (L)` and `Joy-Con (L) (IMU)`.

A Pro Controller also works (left stick, L, ZL, D-pad; USB or Bluetooth): set
`device_name: Pro Controller` under `amw_joycon` in teleop.yaml.

## Test on the mock arm (RViz)

Terminal 1:

```bash
amw build
amw start mock
```

Lay the Joy-Con on the table until the terminal shows `[amw_joycon] Ready. Gyro bias ... stick centre ...`,
and wait for `Ready. The arm starts OFF` from amw_teleop. RViz opens with the arm.

Terminal 2 (raw input, no arm motion):

```bash
amw joystick
```

| Do | Expect |
| --- | --- |
| nothing | `stick x +0.00 y +0.00`, `held -`, `IMU ~200 Hz`, gyro within about +-0.3 deg/s |
| stick right, then up | x goes to about +1.0, then y to about +1.0 (if up shows -, set `invert_y: false`) |
| press L, ZL, D-pad up/down/right | `held L`, `held ZL`, `held DPAD_UP` ... |
| turn the Joy-Con | gyro numbers move; roughly 0 again when still |
| hold it upright, twist it about its long side | **note which gyro axis moves: that is `hand_long_axis`** |
| hold it upright, tip the top away from you | note the axis: put it first in `hand_axes` |
| hold it upright, tip the top to your right | note the axis: put it second in `hand_axes` |

Ctrl-C, then terminal 2:

```bash
amw status
```

| Do (watch RViz) | Expect |
| --- | --- |
| D-pad right, stick centred | status ON, `[ON] moving (translation)` in terminal 1 |
| stick right / up | tip moves -x / -y in RViz (world axes: show the TF of `world` in RViz if unsure); a small push moves slowly, a full push at 10 mm/s |
| D-pad up a few times, then down | `level` in `amw status` goes 12, 14 ... and back; full stick gets faster / slower |
| hold L, stick up / down | tip goes down / up only, no x/y |
| push the stick, then press L | `MODE_CHANGED` "centre the stick"; let go, it carries on in z |
| hold the Joy-Con upright, press D-pad left | terminal 1: `Joy-Con neutral set (D-pad left)` |
| hold ZL (Joy-Con still upright) | the iron swings slowly to straight down and stops |
| still ZL: tip the Joy-Con top away ~20 deg | the iron tilts ~20 deg about world X; tip it back and it follows |
| still ZL: tip it to the side | the iron tilts about world Y |
| still ZL: twist it about its long side | the iron does not move |
| release ZL | the iron stays where it is; the stick drives x/y again |
| D-pad right | OFF |
| turn the Joy-Con off (hold sync / disconnect) while ON | `[STOP] No controller input ...`; stays OFF after reconnect until D-pad right |

If twisting moves the iron, `hand_long_axis` is wrong; if the iron tilts about the wrong world axis,
swap the two `hand_axes`; if it tilts the opposite way, flip that entry of `hand_signs`. Use the axes
you noted with `amw joystick`, edit teleop.yaml and restart. Large tilts can run a joint into its limit:
the arm then cannot hold the tip still while pointing, and the tip drifts (the same solve as before).
`hand_scale: 0.5` halves the tilt for steadier control.

## If something is wrong

| Symptom | Check |
| --- | --- |
| amw_joycon: `Waiting for "Joy-Con (L)" ... Found: [...]` | is it connected (`bluetoothctl info ...`)? Are the devices readable (udev rule, step 3)? If the list shows `Nintendo Switch Left Joy-Con`, set `device_name` to that. |
| amw_joycon: `No motion data ... joycond` | `systemctl status joycond`; stop it (step 2); reconnect the Joy-Con |
| never says `Ready.` | the Joy-Con must lie still for 1 s; raise `still_deg_s` if your table vibrates |
| ON refused: joystick lost | amw_joycon is silent until `Ready.` and while the Joy-Con is disconnected |
| ZL turns the arm OFF (`IMU_LOST`) | no `/amw/hand_imu`: `amw ros2 topic hz /amw/hand_imu` should show about 200 |

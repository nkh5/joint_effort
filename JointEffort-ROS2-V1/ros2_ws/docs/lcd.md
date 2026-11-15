# Status screen (LCD)

The Pi shows `/amw/status` (published by `amw_teleop`) on the Waveshare **3.5inch RPi LCD (B)**:
ON / OFF / CENTRE STICK / STOPPED with the reason and what to do, a warning strip while a
direction is blocked, the mode and the speed level. It only listens, so it can never move the
arm. If the arm software stops, it shows **NO DATA** within 0.6 s.

The program is `src/amw_teleop/amw_teleop/lcd.py`; it scales to any screen size, so a
different panel (e.g. the 4" HDMI one, 800x480) also works.

## Preview on a laptop (no Pi needed)

```bash
amw lcd --demo                     # keys: o on/off, c centre, s stop, w warning, m mode, +/- speed, j joystick, n no data
amw lcd                            # live, while `amw start` runs (480x320 window; --size 800x480 for others)
```

## Pi setup (once, after flashing Ubuntu Server)

1. **Driver overlay.** Copy `waveshare35b-v2.dtbo` (from Waveshare's 3.5inch RPi LCD (B) driver
   package) to `/boot/firmware/overlays/`, then edit `/boot/firmware/config.txt`:

   ```
   dtparam=spi=on
   dtoverlay=waveshare35b-v2
   #dtoverlay=vc4-kms-v3d          <- comment this out
   ```

   Reboot. Check: `ls /dev/fb*` shows `/dev/fb0`, and `cat /sys/class/graphics/fb0/virtual_size`
   prints `480,320` (if it says `320,480`, use `--rotate 90` below).

2. **Packages and permissions.**

   ```bash
   sudo apt install python3-pygame fonts-dejavu-core
   sudo usermod -aG video jointeffort       # log out and back in
   ```

3. **Try it by hand** (with `amw start` running in another terminal, or not, to see NO DATA):

   ```bash
   amw lcd --fb                  # q or Ctrl-C to stop
   ```

   If a white blinking cursor draws over it, the text console is still bound to the screen:
   `echo 0 | sudo tee /sys/class/vtconsole/vtcon1/bind` (the service below does this itself).

4. **Start it at boot.**

   ```bash
   cd ~/joint_effort/JointEffort-ROS2-V1/ros2_ws
   sudo cp pi/amw-lcd.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now amw-lcd
   journalctl -u amw-lcd -f      # logs
   ```

   The service runs `amw lcd --fb` from `~/joint_effort` (edit `AMW=` in the service file if the
   repository is somewhere else). It retries every 2 s, so it survives the arm software restarting.

## What it shows

| situation | screen |
| --- | --- |
| arm software not running | NO DATA |
| started, OFF | OFF, "Centre the stick and press ON (panel 1)." |
| ON pressed, or mode changed | CENTRE STICK and the reason |
| moving | ON (green); an amber strip if a direction is blocked (joint limit, near a singularity or obstacle) |
| any fault | STOPPED (red), what happened and what to do |
| Pico unplugged | NO CONTROLLER |

The same text is in `amw status` and in the `amw start` terminal; the full list of reasons is in
docs/troubleshooting.md.

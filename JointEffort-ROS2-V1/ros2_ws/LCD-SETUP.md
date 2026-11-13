# AMW status screen: Waveshare 4inch HDMI LCD on the Pi 5

The screen runs on the Pi (`lcd.py`). It shows what the input forwarder publishes on
`/amw/display_state` from the PC that runs `run_input.sh`. The Pi only listens, so it never
affects motion. If the PC, the network or the input launch stops, the screen shows **NO DATA**
within 0.6 s.

Panel: 480x800 (portrait), used sideways as 800x480. Video comes over micro-HDMI. Touch is not
used. Touch would need the SPI pins and a driver.

## 0. Wiring

- **Video:** Pi 5 **HDMI0** (the micro-HDMI port next to the USB-C power socket), through a
  micro-HDMI → HDMI cable to the LCD. Linux calls this port `HDMI-A-1`.
- **Power:** the LCD has no USB power socket. It takes 5 V from its 40-pin header.
  - Plugged straight onto the Pi's header: nothing else to do.
  - Mounted away from the Pi: run jumper wires from Pi pin 2 (5 V) to LCD pin 2, and from Pi
    pin 6 (GND) to LCD pin 6.

## 1. Remove the old 3.5" SPI LCD settings

```bash
sudo nano /boot/firmware/config.txt
```

- Delete or comment out `dtoverlay=waveshare35b-v2`.
- Re-enable KMS by removing the `#` from `dtoverlay=vc4-kms-v3d`. HDMI needs it.
- `dtparam=spi=on` can stay.

```bash
sudo reboot
```

## 2. Check what the Pi sees

```bash
cat /sys/class/graphics/fb*/name
cat /sys/class/graphics/fb0/virtual_size
cat /sys/class/drm/card*-HDMI-A-1/status
cat /sys/class/drm/card*-HDMI-A-1/modes
```

Note down:

| what | expected |
| --- | --- |
| fb name | something with `drm` in it (e.g. `vc4drmfb`) |
| fb0 `virtual_size` | `480,800` |
| HDMI-A-1 status | `connected` |
| HDMI-A-1 modes | `480x800` near the top |

If the size isn't `480,800`, or the screen is blank or garbled, force the mode. Add this to the
**end of the single line** in `/boot/firmware/cmdline.txt`, with a space before it and no new line:

```text
video=HDMI-A-1:480x800M@60D
```

Reboot and run the four commands again. If it's still wrong, send me the output. Waveshare's
own `hdmi_timings` lines are for the old non-KMS firmware and the Pi 5 ignores them.

## 3. Software on the Pi

```bash
sudo apt install -y python3-pygame fonts-dejavu-core
sudo usermod -aG video jointeffort      # framebuffer access; log out and back in afterwards
mkdir -p ~/amw
```

From the PC (in the `ros2_ws` folder):

```bash
scp src/joint_effort_input/joint_effort_input/lcd.py jointeffort@amw:~/amw/lcd.py
```

The Pi also needs ROS 2 (`rclpy`, `std_msgs`), which it already has from the old LCD script.

## 4. Test the screen without ROS

```bash
python3 ~/amw/lcd.py --fb --demo
```

Keys: `space` enabled, `m` mode, `c` centre-stick wait, `+`/`-` speed, `f` servo halt,
`w` servo warning, `j` no controller, `u` unconfirmed mapping, `n` no data, `q` quit.

- Upside down? Add `--rotate 270`.
- Old text console drawing over it or a blinking cursor?
  `echo 0 | sudo tee /sys/class/vtconsole/vtcon1/bind`

Note down: the right `--rotate` value.

## 5. Live with the arm

PC: start `run_real.sh` and `run_input.sh` as usual. Then on the Pi:

```bash
source /opt/ros/jazzy/setup.bash        # the Pi's ROS 2 setup file
export ROS_DOMAIN_ID=88
ros2 topic echo /amw/display_state --once   # must print one JSON message
python3 ~/amw/lcd.py --fb                  # add --rotate 270 if needed
```

If `topic echo` prints nothing:

- Pi and PC must be on the same network and subnet.
- Fedora's firewall usually blocks ROS 2 (DDS) discovery. Trust the Pi for this session:
  `sudo firewall-cmd --zone=trusted --add-source=<pi-ip>`. Add `--permanent` and run
  `sudo firewall-cmd --reload` to keep it.
- If the Pi runs an older ROS 2 (Humble), make sure `ROS_LOCALHOST_ONLY` is unset or `0`.

Check each state on the screen:

| action | screen |
| --- | --- |
| Pico OFF | DISABLED, "Ready - press ON (panel 1)" |
| panel 1 ON | ENABLED |
| stick button | CENTRE STICK, mode band flips |
| let go of the stick | ENABLED again, new mode |
| encoder | SPEED number and bar |
| unplug the Pico | NO CONTROLLER |
| Ctrl-C `run_input.sh` | NO DATA |

## 6. Start it at boot

Edit `ROS_SETUP` and the `lcd.py` path in `pi/amw-lcd.service` if they differ, then:

```bash
scp pi/amw-lcd.service jointeffort@amw:~/amw/          # from the PC
sudo cp ~/amw/amw-lcd.service /etc/systemd/system/     # on the Pi
sudo systemctl daemon-reload
sudo systemctl enable --now amw-lcd
journalctl -u amw-lcd -f                                # check it
```

The service unbinds the text console itself and restarts the screen if it crashes. If you
need `--rotate 270`, add it to the `ExecStart` line.

## Preview on the PC

```bash
./scripts/run_lcd.sh            # live window, same data as the Pi
./scripts/run_lcd.sh --demo     # no ROS
```

This needs pygame inside the distrobox: `sudo apt install python3-pygame`.

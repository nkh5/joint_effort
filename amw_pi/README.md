# AMW Pi Starter

This simplified package contains one complete Python program, `amw_pi.py`, and this guide. The program implements serial packets, scanning, ID assignment, position reading, a small position move and torque disable in one file. The only external Python dependency is `pyserial`. There are no wrapper scripts or bundled SDK folders.

## Your immediate task

Run the program on the Raspberry Pi 5, find and assign motor IDs, then demonstrate controlled movement through the Waveshare adapter. Your broader role also includes receiving Pico inputs, but the JSON reader and ROS integration are not implemented in this version.

## Where Ubuntu goes

Ubuntu runs on the Pi 5. Its OS files live on your SanDisk 64 GB microSD card, which is the Pi's boot storage. You do not need an SSD for this milestone. Use Raspberry Pi Imager on your laptop to write the Pi 5 Ubuntu 24.04 LTS ARM64 image to that card. This erases the selected card; it does not install Ubuntu as your laptop's operating system. Insert the card into the powered off Pi, then boot and complete setup.

If the Pi already has a working Linux installation, the motor script can be tested there first with Python 3 and pyserial. Ubuntu 24.04 is the proposed project OS for the planned ROS 2 Jazzy integration, not a requirement of this serial motor script itself.

## Hardware connections

| Part | Connection and purpose |
| --- | --- |
| SanDisk microSD | Pi card slot; stores Ubuntu and your files |
| Yahboom PD power board | Correctly rated DC input; regulated power output to Pi power input |
| Waveshare Bus Servo Adapter A | USB data cable from Pi USB port to adapter; jumper B for USB control |
| STS3215 motor | Servo bus cable from adapter to one motor during initial setup |
| Servo supply | Adapter motor power input at the motor's rated voltage; 12 V for your stated 12 V variant |
| Pico | Separate USB data connection to Pi when testing inputs later |

The Yahboom board supplies Pi power. The Waveshare adapter carries motor communication. The motors need their external supply. Check connector polarity and D/V/G labels before powering up. Do not use the Yahboom's regulated 5 V output as the 12 V motor supply. Turn motor power off before changing servo cables.

## Set up Python on the Pi

Save `amw_pi.py` in a working folder on the Pi and open a terminal in that folder:

```bash
sudo apt update
sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install pyserial==3.5
python amw_pi.py --help
```

You can type commands on the Pi with a monitor and keyboard. SSH is optional. To use SSH, install `openssh-server`, enable its service, find the Pi's IP address with `hostname -I`, and connect from your laptop using `ssh YOUR_USERNAME@PI_IP_ADDRESS`.

Find the Waveshare serial port:

```bash
python -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/
```

Use the actual adapter path. Prefer its stable `by-id` path; `/dev/ttyUSB0` below is only an example. Identify the Waveshare port separately from the Pico port.

```bash
AMW_PORT=/dev/ttyUSB0
```

If Ubuntu reports permission denied, run `sudo usermod -aG dialout "$USER"`, log out, then log back in and activate the virtual environment again. Close any other programs using the adapter. Default host baud is 1,000,000; `--baud` changes the host connection only and must match the servo baud.

## Commands

### Scan and read

```bash
python amw_pi.py --port "$AMW_PORT" scan
python amw_pi.py --port "$AMW_PORT" read --id 1
```

A full scan takes about 25 seconds when most IDs are absent. It prints responding IDs from 1 through 253. Replace `1` in later commands with the observed motor ID. Reading prints encoder counts for three seconds by default; use `--seconds 5` for five seconds. Communication faults stop the command with an error.

### Assign an ID

Connect one physical, unloaded motor only. A scan cannot reliably distinguish two motors sharing an ID. Preview and then write:

```bash
python amw_pi.py --port "$AMW_PORT" id --id 1 --new-id 3
python amw_pi.py --port "$AMW_PORT" id --id 1 --new-id 3 --single --execute
```

The program disables torque, unlocks EEPROM, sends the ID write once, verifies the new register and relocks EEPROM. Label the motor, power cycle it, and run `read --id 3` to verify persistence. A failed ID change should be investigated with the motor still isolated rather than blindly repeated. Assignment to the current ID performs no write.

The proposed team convention remains left arm IDs 1 through 5 and right arm IDs 11 through 15. Agree on physical joint order before labeling motors by joint name.

### Move a motor

Use a secured, unloaded motor with no soldering tool attached. If torque is already enabled, first disable it on that bench setup:

```bash
python amw_pi.py --port "$AMW_PORT" off --id 3 --bench
```

Preview a small offset, then execute:

```bash
python amw_pi.py --port "$AMW_PORT" move --id 3 --delta 32
python amw_pi.py --port "$AMW_PORT" move --id 3 --delta 32 --bench --execute
```

The program checks position mode and torque state, reads the current position, seeds the current position as the goal, enables torque and commands the offset. It prints measured position and then disables torque. It also attempts torque disable after exceptions and Ctrl+C. If communication is lost, software cannot guarantee torque disable; cut bench motor power if disable is unverified.

This command produces a small position change, not continuous spinning. It does not return automatically to the starting position. Defaults are an interior range of 100 through 3995 counts, offsets up to 64 counts, and fixed vendor speed and acceleration values of 100 and 10. These limits are for the bench demonstration and do not replace calibrated joint limits. Do not use this torque release behavior on a gravity loaded arm.

## Pico JSON status

This proposed message is not consumed by the program:

```json
{"seq":42,"x":3200,"y":2048,"speed":3000,"enable":true,"arm":"left"}
```

A later Pi input reader must parse complete messages, validate ranges and freshness, apply input calibration, and interpret enable, speed and arm selection. The input format and update timing still need agreement with the Pico firmware author. No joystick movement is wired directly into this bench tool.

## Verification and next demonstration

The new single file passed 24 automated software tests using an emulated serial transport. They covered packet bytes, fragmented responses, checksum errors, timeouts, servo faults, ID verification and relocking, movement limits, stale goal replacement and torque cleanup. No physical Pi or servo was connected during these checks. The old package's test count and simulation commands do not apply to this replacement.

Record your real demonstration results:

| Check | Result |
| --- | --- |
| Pi boots from microSD and runs Python | Not yet recorded |
| Waveshare adapter is identified | Not yet recorded |
| One motor responds and returns position | Not yet recorded |
| New ID survives a power cycle | Not yet recorded |
| Small unloaded move shows measured feedback | Not yet recorded |
| Normal exit and Ctrl+C disable bench torque | Not yet recorded |

## References

* [Ubuntu installation on Raspberry Pi](https://ubuntu.com/hardware/docs/boards/tutorials/raspberry-pi-other/)
* [Waveshare Adapter A wiring](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example)
* [Yahboom Pi 5 power board](https://category.yahboom.net/products/power-board-pi5)
* [FEETECH register and protocol reference](https://github.com/ftservo/FTServo_Python/tree/cbcfa64674d592f7e5028ae72af42580f60500b4/scservo_sdk)

The script contains an original minimal packet implementation checked against the referenced vendor SDK. It is limited to this STS3215 position mode bench workflow.

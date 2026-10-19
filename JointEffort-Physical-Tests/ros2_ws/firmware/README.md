# Pico physical input proposal

Based on the team's `JackTestV1/Pico_Controller.ino` at commit
`7e63e7a929292e6bf87458223642f0ec8ba5b048`. The only functional addition is a
heartbeat on unused HID button 10, changing every 25 ms. The GPIO assignments
are unchanged. Check them against the actual panel before building or flashing.

Use the same Arduino Pico core and Adafruit TinyUSB settings as the working
team firmware. This proposed copy has not been compiled or flashed here.
`tinyusb.h` comes from the team's `Pico_Controller/` folder and keeps its MIT
license notice. The unchanged upstream references are in `hardware_reference/`.

In the physical ROS profile, button 0 must be held to move, virtual button 7 or
8 must also grant permission, and button 9 selects mode. Host autorepeat cannot
substitute for a changing heartbeat. See `HARDWARE.md` before using the arm.

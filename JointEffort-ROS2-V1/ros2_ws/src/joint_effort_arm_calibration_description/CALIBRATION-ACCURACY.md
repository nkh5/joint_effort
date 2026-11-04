# Matching the Joint Effort arm to its URDF

The CAD establishes a nominal reference. Physical calibration establishes how the
assembled arm and its encoder readings relate to that reference. Both are needed.
Use the new calibration package only after its native build and independent checks
have passed; the earlier natural-zero package remains on calibration hold.

## The reference pose

The fixed base defines +Z up, +X forward and +Y left. At all five joint values zero,
the actual upper link is vertical, the forearm is horizontal forward, and the iron
points vertically down. Joint positions refer to the motor shaft axes. Link
directions refer to the long straight edges of the native CAD links.

A joint axis is a line. Its coordinate origin can sit at a gear reference plane
or another point along that same line. Two gear origins can therefore have
different positions along their shaft axes even when the shafts are parallel
and the printed link is vertical. A line drawn between those origins is not the
alignment measurement for the printed link. Check shaft directions and the
native link edges instead; those are the datums used by the new verification.

| Joint | Axis at the reference pose | Positive rotation, viewed in the base frame |
| --- | --- | --- |
| Base rotation | +Z | Forward rotates toward left |
| Shoulder | +Y | Vertical upper link rotates toward forward |
| Elbow | +Y | Forward forearm rotates toward down |
| Wrist yaw | -Z | Forward rotates toward right |
| Wrist pitch | -Y | Downward iron rotates toward forward |

These are ROS direction conventions. The corresponding encoder direction signs
must be measured on the actual motors. The fourth motor has a vertical shaft at
this reference pose; it should be mapped as wrist yaw.

## Prepare the mechanical assembly first

The corrected CAD proposal levels the shoulder and elbow shafts using shaft and
mounting-hole geometry. It includes a shallow clearance pocket in the shoulder
motor mount and the upper link's elbow motor cradle. Each nominal floor is about
4.57 mm thick after relief. Confirm the new pockets, motor seating and screw
engagement physically; the CAD clearance test does not establish printed fit or
fastener strength. The original mated assembly is preserved.

Avoid aligning a calibration fixture to a molded case face merely because it looks
flat. The imported motor case includes 0.8-degree draft faces. Mounting datums and
the shaft axis provide the geometric references for the corrected model.

## Calibrate the motors and the assembled arm

1. Use the motor/protractor fixture to identify each servo, its encoder count
   direction, and repeatability. Record readings rather than assuming every
   servo's catalog center count corresponds to the arm's assembled zero.
2. Reinstall the motors, horns and links in the intended orientation. Horn
   installation and mounting clearances can change the assembled zero established
   by a test of a bare motor.
3. Support the arm, keep the iron cold, level the base, and place the actual links
   in the reference pose. Use a square or angle gauge on the links. Check the wrist
   orientation against the CAD view as well as checking the iron direction.
4. Record one zero encoder count for each assembled joint. Fill in the servo IDs,
   zero counts and direction signs in the package's calibration template. Preserve
   the raw measurements so the process can be repeated after reassembly.
5. Convert readings using:

   `joint_angle_rad = direction * (unwrapped_count - zero_count) * 2*pi/4096`

   This assumes the direct servo-to-link arrangement and nominal 4096-count
   encoder. Handle count wraparound consistently in the ROS driver. A sign is +1
   or -1, established by the small-movement direction test.
6. Check several nonzero poses against independent measurements. Include single
   joint moves and combined moves. Approach test poses from both directions to
   reveal backlash. A good zero-pose match alone does not verify the whole model.
7. Measure the actual iron tip position relative to the holder. The mock iron's
   nominal 221 mm reference is consistent with its CAD tip, but it is not a
   physical tool-center-point measurement. Repeat this step after changing tips
   or repositioning the iron in its clamps.

## What determines the final accuracy

The nominal encoder resolution is 360/4096 = 0.087890625 degrees per count. A
half-count corresponds to about 0.04395 degrees before accounting for encoder
accuracy, backlash, or mechanical play. At a 300 mm lever arm, a 0.1-degree angle
error is roughly 0.52 mm of transverse displacement. Errors from different joints
combine according to the arm's pose.

Printed dimensions, bearing clearance, horn fit, shaft alignment, cable forces,
link flex and iron clamp position also matter. Record the measurement instrument
and repeatability with the calibration data. Numeric CAD alignment to near zero
does not mean the physical arm has that same angular accuracy.

Choose actual travel limits after checking the arm and cables through their usable
range. The package's +/-30-degree display ranges are placeholders. Supply voltage,
operating speed, continuous loads and dynamics require separate confirmation;
this export does not enable a motor controller or change servo firmware.

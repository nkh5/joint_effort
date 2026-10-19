# Team motor feedback and trajectory preview

Imported from the team repository at commit
`7e63e7a929292e6bf87458223642f0ec8ba5b048`.
The Python source is unchanged. It supplies calibration conversions, read-only
feedback and trajectory previews. Its `amw_motors.py` uses an external FTServo
SDK for its optional commands. The primary motor test in the top-level README
uses pyserial and does not need that SDK. Do not run competing serial owners.
There is one copy of this ROS package in this workspace.

# CAD visual model

This package holds the imported URDF and six visual meshes.

Use the project iron-arm launch for mock control. For the separate visual preview:

```bash
ros2 launch joint_effort_arm_draft_description display.launch.py
```

This package has no motor interface or collision model. The saved pose, joint limits, and tool offset still need physical checks. The iron package adds the control and collision settings.

The package contact and license fields are placeholders.

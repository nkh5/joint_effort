from glob import glob
from setuptools import setup
setup(name='joint_effort_input', version='0.1.0', packages=['joint_effort_input'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/joint_effort_input']),
                  ('share/joint_effort_input', ['package.xml'])] +
                 [(f'share/joint_effort_input/{d}', glob(f'{d}/*')) for d in ['config', 'launch']],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['pico_forwarder = joint_effort_input.forwarder:main',
                                       'display_preview = joint_effort_input.display:main',
                                       'joy_teleop_clean = joint_effort_input.teleop:main']})

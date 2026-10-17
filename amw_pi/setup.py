from glob import glob
from setuptools import setup

setup(name='amw_pi_bridge', version='0.1.0', packages=['amw_pi_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/amw_pi_bridge']),
                  ('share/amw_pi_bridge', ['package.xml'])] +
                 [(f'share/amw_pi_bridge/{d}', glob(f'{d}/*')) for d in ['config', 'launch']],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'motor_feedback = amw_pi_bridge.feedback:main',
          'trajectory_preview = amw_pi_bridge.preview:main',
          'interface_demo = amw_pi_bridge.demo:main']})

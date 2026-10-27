from setuptools import setup
from glob import glob
setup(name='joint_effort_iron',version='0.1.0',packages=['joint_effort_iron'],
 data_files=[('share/ament_index/resource_index/packages',['resource/joint_effort_iron']),('share/joint_effort_iron',['package.xml'])]+[(f'share/joint_effort_iron/{d}',glob(f'{d}/*')) for d in ['config','urdf','launch','meshes']],
 install_requires=['setuptools'],zip_safe=True,entry_points={'console_scripts':['iron_task_bridge = joint_effort_iron.bridge:main','iron_datum_sliders = joint_effort_iron.datum_sliders:main','iron_jog = joint_effort_iron.jog_joint:main']})

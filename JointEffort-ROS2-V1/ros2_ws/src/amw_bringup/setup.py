from glob import glob
from setuptools import setup

setup(
    name='amw_bringup', version='1.0.0', packages=['amw_bringup'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/amw_bringup']),
                ('share/amw_bringup', ['package.xml']),
                ('share/amw_bringup/launch', glob('launch/*.py')),
                ('share/amw_bringup/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True)

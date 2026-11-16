from setuptools import setup

setup(
    name='amw_teleop', version='1.0.0', packages=['amw_teleop'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/amw_teleop']),
                ('share/amw_teleop', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'amw_teleop = amw_teleop.node:main',
        'amw_lcd = amw_teleop.lcd:main',
        'amw_joycon = amw_teleop.joycon:main',
    ]})

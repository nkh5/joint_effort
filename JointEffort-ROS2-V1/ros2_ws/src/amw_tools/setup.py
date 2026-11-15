from setuptools import setup

setup(
    name='amw_tools', version='1.0.0', packages=['amw_tools'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/amw_tools']),
                ('share/amw_tools', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': ['preflight = amw_tools.preflight:main']})

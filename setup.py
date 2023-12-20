from setuptools import setup

package_name = 'diablo_ctrl_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    py_modules=['diablo_ctrl_gui.diablo_ctrl_gui'],
    maintainer='Omar Mostafa',
    maintainer_email='omm7813@nyu.edu',
    description='GUI for controlling Diablo Robot for ROS2 using PyQt5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ctrl_gui_node = diablo_ctrl_gui.main:main'
        ],
    },
)

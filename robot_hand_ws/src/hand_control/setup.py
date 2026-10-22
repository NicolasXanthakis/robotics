"""Install the hand demonstration package and visualization assets."""

from glob import glob

from setuptools import find_packages, setup

setup(
    name='hand_control',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hand_control']),
        ('share/hand_control', ['package.xml']),
        ('share/hand_control/launch', glob('launch/*.launch.py')),
        ('share/hand_control/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nicol',
    maintainer_email='nicol@todo.todo',
    description='Joint motion demonstration and visualization for the robotic hand.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'test_joint_publisher = hand_control.test_joint_publisher:main',
        ],
    },
)

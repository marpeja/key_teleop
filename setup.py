from setuptools import setup
import os
from glob import glob
from setuptools import setup

package_name = 'key_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaime',
    maintainer_email='Jaime@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'key_teleop = key_teleop.key_teleop:main',
            'key_teleop_2 = key_teleop.key_teleop_explained:main',
            'key_teleop_3 = key_teleop.key_teleop_listener:main',
        ],
    },
)

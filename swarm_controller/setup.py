import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_controller'
os.path.join(package_name, 'submodules')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='artemkondratev5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_controller = swarm_controller.swarm_controller:main',
            'swarm_acc_mpc_node = swarm_controller.swarm_acc_mpc_node:main',
            'swarm_acc_kin_mpc_node = swarm_controller.swarm_acc_kin_mpc_node:main',
            'peer_localization = swarm_controller.peer_localization:main',
            'pacemaker_controller = swarm_controller.pacemaker_controller:main',
            'simulator = swarm_controller.simulator:main',
        ],
    },
)

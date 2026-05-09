from setuptools import find_packages, setup

package_name = 'trajectory_drawer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='artemkondratev5@gmail.com',
    description='Simple matplotlib GUI to draw pacemaker waypoint trajectories',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw = trajectory_drawer.draw:main',
        ],
    },
)

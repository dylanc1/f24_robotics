from setuptools import find_packages, setup

package_name = 'outdoor_simulation_project'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/daytime_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/dusk_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/rec_center_fields_daytime.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/rec_center_fields_dusk.wbt']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot_webots.urdf',
    'resource/ros2control.yml',
]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='dylancanipe6@gmail.com',
    description='Simulation for CS 560 Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],    
        'console_scripts': ['outdoor_simulation_project = outdoor_simulation_project.outdoor_simulation_project:main']
    },
)

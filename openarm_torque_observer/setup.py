from setuptools import find_packages, setup

package_name = 'openarm_torque_observer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/torque_observer.launch.py']),
        ('share/' + package_name + '/config', ['config/friction_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenArm Dev',
    maintainer_email='dev@openarm.dev',
    description='Shadow-mode torque observer for feedforward validation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'torque_observer = openarm_torque_observer.torque_observer_node:main',
        ],
    },
)

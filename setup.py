from glob import glob

from setuptools import find_packages, setup

package_name = 'picking_example'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE', 'CHANGELOG.rst']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/real', glob('config/real/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuel beschi',
    maintainer_email='manuel.beschi@unibs.it',
    description='Pick and place example in the Sharework cell (Gazebo).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_and_place = picking_example.pick_and_place:main',
            'sim_grasp_manager = picking_example.sim_grasp_manager:main',
        ],
    },
)

from glob import glob

from setuptools import setup

package_name = 'ur_ignition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vatan Aksoy Tezer',
    maintainer_email='vatan@picknik.ai',
    description='A package that includes necessary launch files and control scripts to get UR robots working with Ignition Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_ignition_control = ur_ignition.ur_ignition_control:main'
        ],
    },
)

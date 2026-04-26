from setuptools import find_packages, setup

package_name = 'ow_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albertolr98',
    maintainer_email='alberto.lopezro@alumnos.upm.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'linear_ramp = ow_control.linear_ramp:main',
            'cmd_vel_relay = ow_control.cmd_vel_relay:main',
            'trajectory_controller = ow_control.trajectory_controller:main',
        ],
    },
)

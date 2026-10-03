from glob import glob
from setuptools import setup

package_name = 'rtk_gps_um982'
um982_src = 'third_party/UM982-RTK-GPS-Library/um982'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'um982'],
    package_dir={
        'um982': um982_src,
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/rtk_gps_um982.launch.py']),
        ('share/' + package_name + '/config',
         ['config/default.yaml']),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
        ('share/' + package_name + '/tools', glob('tools/*.sh') + glob('tools/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='nkb',
    maintainer_email='nakaba_tokutoku@hotmail.com',
    description='ROS 2 driver for Unicore UM982 dual-antenna RTK GNSS.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtk_gps_um982_node = rtk_gps_um982.driver_node:main',
            'prepare_clock_host = rtk_gps_um982.clock_setup:main',
            'ptp_trial = rtk_gps_um982.ptp_trial:main',
        ],
    },
)

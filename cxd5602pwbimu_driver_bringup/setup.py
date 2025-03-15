"""Setup."""

from glob import glob

from setuptools import setup

package_name = 'cxd5602pwbimu_driver_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/{}/launch'.format(package_name), glob('launch/*.launch.py')),
        ('share/{}/rviz'.format(package_name), glob('rviz/*.rviz')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isato',
    maintainer_email='ray255ar@gmail.com',
    description='cxd5602pwbimu launch file package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
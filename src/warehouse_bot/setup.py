from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arne',
    maintainer_email='fx.zype@gmail.com',
    description='Package for an autonomous warehouse bot based on turtlebot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # here, the entry points for nodes that belong to this package are defined
            'scan_filter = warehouse_bot.scan_filter:main',
            'warehouse_bot_main = warehouse_bot.warehouse_bot_main:main'
        ],
    },
)

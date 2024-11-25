from setuptools import find_packages, setup

package_name = 'warehouse_bot'

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
    maintainer='Arne',
    maintainer_email='fx.zype@gmail.com',
    description='Package for an autonomous warehouse bot based on turtlebot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # here, the nodes for this package are defined
            'talker = warehouse_bot.publisher_member_function:main',
            'listener = warehouse_bot.subscriber_member_function:main',
        ],
    },
)

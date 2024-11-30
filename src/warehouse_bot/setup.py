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
            # here, the entry points for nodes that belong to this package are defined
            'scan_filter = warehouse_bot.scan_filter:main',
            'scan_listener = warehouse_bot.scan_listener:main',
            'service = warehouse_bot.service_member_function:main',
            'client = warehouse_bot.client_member_function:main',
            'publisher = warehouse_bot.publisher_member_function:main',
            'subscriber = warehouse_bot.subscriber_member_function:main',
            'address_publisher = warehouse_bot.adress_book_publisher:main',
            'fib_action_server = warehouse_bot.fibonacci_action_server:main',
            'fib_action_client = warehouse_bot.fibonacci_action_client:main'
        ],
    },
)

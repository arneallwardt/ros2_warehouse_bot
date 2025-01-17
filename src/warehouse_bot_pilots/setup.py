from setuptools import find_packages, setup

package_name = 'warehouse_bot_pilots'

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
    maintainer='kilab',
    maintainer_email='arne.allwardt@th-brandenburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'product_aligner = warehouse_bot_pilots.product_aligner:main',
            'product_gripper = warehouse_bot_pilots.product_gripper:main',
        ],
    },
)

from setuptools import setup

package_name = 'ball_placement_strategy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stitch',
    maintainer_email='1528994924@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_placement_node = ball_placement_strategy.ball_placement_node:main',
            'basket_status_publisher = ball_placement_strategy.basket_status_publisher:main',
        ],
    },
)

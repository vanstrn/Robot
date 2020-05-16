from setuptools import setup

package_name = 'pirobot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neale',
    maintainer_email='nealevanstrn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor = pirobot_base.motorNode:main',
            'controller = pirobot_base.controllerNode:main',
            'imu = pirobot_base.imuNode:main',
            'gps = pirobot_base.gpsNode:main',
            'cont = pirobot_base.contNode:main',
        ],
    },
)

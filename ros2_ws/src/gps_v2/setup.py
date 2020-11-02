from setuptools import setup

package_name = 'gps_v2'

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

    #Add entrypoints by with the following syntax: #############################
    # 'executable_name = path.to.file.fileName:functionName'
    ############################################################################
    entry_points={
        'console_scripts': [
            'gps = gps_v2.gpsNode:main',
        ],
    },
)

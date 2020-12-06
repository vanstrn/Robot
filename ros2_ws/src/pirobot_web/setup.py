from setuptools import setup
import os
package_name = 'pirobot_web'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    package_data={
                    '':package_files('pirobot_web/templates')
                    },

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
            'web = pirobot_web.flaskNode:main',
        ],
    },
)

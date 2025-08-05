import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robethichor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nele Reichert',
    maintainer_email='reichern@uni-bremen.de',
    description='RobEthiChor ROS2: Interruption Scenario Expansion',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ethics_manager_node = robethichor.nodes.ethics_manager.ethics_manager_node:main',
            'context_manager_node = robethichor.nodes.context_manager.context_manager_node:main',
            'negotiation_manager_node = robethichor.nodes.negotiation_manager.negotiation_manager_node:main',
            'mission_controller_node = robethichor.nodes.mission_controller.mission_controller_node:main',
            'connector_node = robethichor.nodes.connector.connector_node:main',
            'interruption_manager_node = robethichor.nodes.interruption_manager.interruption_manager_node:main',
        ],
    },
)

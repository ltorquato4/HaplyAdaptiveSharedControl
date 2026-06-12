from setuptools import setup, find_packages
from glob import glob

package_name = 'haply_interface'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='kiss.gergely1213@edu.bme.hu',
    description='Haply ROS2 interface package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse3_driver_node = haply_interface.drivers.inverse3_driver_node:main',
            'handle_driver_node = haply_interface.drivers.handle_driver_node:main',
            'haply_driver_node = haply_interface.drivers.haply_driver_node:main',

            'target_position_sinus = haply_interface.demo_nodes.target_position_sinus:main',
            'target_position_input = haply_interface.demo_nodes.target_position_input:main',
            'PID_test = haply_interface.demo_nodes.PID_test:main',
            'haptic_ball = haply_interface.demo_nodes.haptic_ball:main',
            'haptic_ball_with_damping = haply_interface.demo_nodes.haptic_ball_with_damping:main',
            
            'state_subscriber_haply = haply_interface.demo_nodes.state_subscriber_haply:main',
            'state_subscriber_inverse3 = haply_interface.demo_nodes.state_subscriber_inverse3:main',
            'state_subscriber_handle = haply_interface.demo_nodes.state_subscriber_handle:main',

            'rviz_visualization_node = haply_interface.visualize.rviz_visualization_node:main',
            'plotter_node = haply_interface.visualize.plotter_node:main',

            'daVinci_control = haply_interface.demo_nodes.daVinci_control:main',
        ],
    },
)

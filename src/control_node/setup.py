from setuptools import find_packages, setup

package_name = 'control_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools', 
        'numpy',
        'casadi'
    ],
    zip_safe=True,
    maintainer='Sadegh Khalili Tehrani',
    maintainer_email='sadegh.tehrani@student.kit.edu',
    description='Control Node including adaptive and fixed contol, MPC or state feedback, for Haply shared control experiments.',
    license='Unlicensed',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_node = control_node.control_node:main',
        ],
    },
)

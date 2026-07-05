from setuptools import find_packages, setup

package_name = "test_data_logger"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sadegh Khalili Tehrani",
    maintainer_email="sadegh.tehrani@student.kit.edu",
    description="Test Node for Data Logger for Haply shared-control experiments",
    license="UNLICENSED",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "test_data_logger_node = test_data_logger.test_node_data_logger:main",
        ],
    },
)

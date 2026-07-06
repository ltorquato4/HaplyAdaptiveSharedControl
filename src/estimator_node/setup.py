from setuptools import find_packages, setup

package_name = "estimator_node"

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
    maintainer="root",
    maintainer_email="ines.boujnah@student.kit.edu",
    description="Estimator Node providing real-time estimation of "
    "human control authority using the Recursive Least Squares "
    "(RLS) algorithm for an adaptive haptic shared controller.",
    license="Unlicensed",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "estimator_node = estimator_node.estimator_node:main",
        ],
    },
)

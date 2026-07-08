"""Package configuration for the Haply study GUI."""

from glob import glob

from setuptools import find_packages, setup

package_name = "haply_study_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Luisa Torquato Niño",
    maintainer_email="ltorquato@users.noreply.github.com",
    description="Study GUI for Haply shared-control experiments.",
    license="UNLICENSED",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "test_inverse3_state_topic = "
                "haply_study_gui.tests.test_inverse3_state_topic:main"
            ),
            "study_gui = haply_study_gui.study_gui_node:main",
        ],
    },
)

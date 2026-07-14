"""Package configuration for study orchestration nodes."""

from glob import glob

from setuptools import find_packages, setup

package_name = "study_orchestration"

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
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Luisa Torquato Niño",
    maintainer_email="ltorquato@users.noreply.github.com",
    description=(
        "Scenario rollout and input mapping nodes for Haply shared-control studies."
    ),
    license="UNLICENSED",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "experiment_mapper = study_orchestration.experiment_mapper_node:main",
            "scenario_generator = study_orchestration.scenario_generator_node:main",
        ],
    },
)

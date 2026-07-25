import os
from glob import glob

from setuptools import find_packages, setup

package_name = "study_analysis"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Luisa Torquato Niño",
    maintainer_email="ltorquato@users.noreply.github.com",
    description="Study analysis and deterministic validation tools.",
    license="UNLICENSED",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "analyze_session = study_analysis.cli:main",
            "run_benchmark = study_analysis.benchmark:main",
        ],
    },
)

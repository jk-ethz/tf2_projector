import os
from glob import glob
from setuptools import find_packages, setup

package_name = "tf2_projector"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jk-ethz",
    maintainer_email="ethz@juliankeller.net",
    description="This package provides a node that projects the transform of a given tf2 frame (source) to another tf2 frame (target) under the same root, using an arbitrary (transitive) parent of the target frame as an attachment frame for the projection.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "tf2_projector = tf2_projector.tf2_projector:main",
        ],
    },
)

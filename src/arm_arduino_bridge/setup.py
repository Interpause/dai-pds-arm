import os
from glob import glob

from setuptools import find_packages, setup

package_name = "arm_arduino_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="42513874+Interpause@users.noreply.github.com",
    description="Interface to Arduino controlling arm",
    license="License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = arm_arduino_bridge.bridge:main",
            "direct = arm_arduino_bridge.direct:main",
        ],
    },
)

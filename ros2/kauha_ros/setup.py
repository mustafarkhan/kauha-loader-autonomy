from glob import glob

from setuptools import find_packages, setup

package_name = "kauha_ros"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mustafa R. Khan",
    description="ROS 2 interface for the KAUHA wheel-loader autonomy retrofit",
    license="Proprietary",
    entry_points={"console_scripts": [
        "twin_bridge = kauha_ros.twin_bridge:main",
        "can_bridge = kauha_ros.can_bridge:main",
        "straight_driver = kauha_ros.straight_driver:main",
    ]},
)

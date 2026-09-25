from glob import glob

from setuptools import find_packages, setup

package_name = "superdex_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team 6",
    maintainer_email="team6@example.com",
    description="A ROS 2 interface to Project SuperDex.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = superdex_ros2.sim_node:main",
        ],
    },
)
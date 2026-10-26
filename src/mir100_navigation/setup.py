from glob import glob
from setuptools import find_packages, setup


package_name = "mir100_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/behavior_trees", glob("behavior_trees/*.xml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Local Workspace",
    maintainer_email="local@example.com",
    description="Nav2 and SLAM launch configuration for the MiR100 Gazebo simulation.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "send_goal = mir100_navigation.goal_sender:main",
        ],
    },
)

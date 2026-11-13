from glob import glob
from setuptools import setup


package_name = "mocap_pose_control"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Package maintainer",
    maintainer_email="maintainer@example.com",
    description="NOKOV pose feedback and planar chassis control.",
    license="TODO",
    entry_points={"console_scripts": [
        "pose_controller = mocap_pose_control.controller_node:main",
        "send_goal = mocap_pose_control.send_goal:main",
    ]},
)

from setuptools import setup


package_name = "robokit_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "API_USAGE.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Package maintainer",
    maintainer_email="maintainer@example.com",
    description="ROS 2 Twist bridge for Robokit TCP motion API 2010.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "robokit_cmd_vel = robokit_control.robokit_cmd_vel:main",
            "robokit_tcp = robokit_control.robokit_tcp:main",
        ],
    },
)

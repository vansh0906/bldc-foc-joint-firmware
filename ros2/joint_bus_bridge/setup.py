from setuptools import setup

setup(
    name="joint_bus_bridge",
    version="1.0.0",
    packages=["joint_bus_bridge"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/joint_bus_bridge"]),
                ("share/joint_bus_bridge", ["package.xml"]),
                ("share/joint_bus_bridge/launch", ["launch/bridge.launch.py"])],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vansh Gala",
    description="ROS 2 bridge for BLDC FOC joints on an RS-485 joint bus",
    license="MIT",
    entry_points={"console_scripts": ["bridge = joint_bus_bridge.node:main"]},
)

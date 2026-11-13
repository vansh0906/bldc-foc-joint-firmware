from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package="joint_bus_bridge", executable="bridge", name="joint_bus_bridge",
        parameters=[{"port": "/tmp/jointbus", "joint_ids": [1, 2], "joint_names": ["hip", "knee"],
                     "mode": "impedance", "kp": 2.0, "kd": 0.02, "rate_hz": 500.0}])])

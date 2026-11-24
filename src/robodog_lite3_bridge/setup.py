from glob import glob
from setuptools import find_packages, setup

package_name = "robodog_lite3_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobDog Nav2 maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Read-only Lite3 ROS 2 topic contract monitor and synthetic test publisher.",
    license="UNSPECIFIED",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lite3_state_mock = robodog_lite3_bridge.mock_state_publisher:main",
            "lite3_topic_contract_monitor = robodog_lite3_bridge.topic_contract_monitor:main",
        ],
    },
)

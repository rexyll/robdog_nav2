from setuptools import find_packages, setup

package_name = "robodog_gnss_time"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobDog Nav2 maintainers",
    maintainer_email="maintainers@example.invalid",
    description="ROS 2 GNSS runtime and host time synchronization evidence monitor.",
    license="UNSPECIFIED",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gnss_receiver_time_node = robodog_gnss_time.gnss_receiver_time_node:main",
            "gnss_observation_node = robodog_gnss_time.gnss_observation_node:main",
            "gnss_rtk_quality_node = robodog_gnss_time.gnss_rtk_quality_node:main",
            "host_time_sync_node = robodog_gnss_time.host_time_sync_node:main",
            "robodog_linuxptp_host_evidence = robodog_gnss_time.linuxptp_host_evidence:main",
        ],
    },
)

from setuptools import find_packages, setup

package_name = "robodog_gnss_quality"

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
    description="Transport-independent RTK quality gate core.",
    license="UNSPECIFIED",
    tests_require=["pytest"],
    extras_require={"test": ["pytest"]},
)

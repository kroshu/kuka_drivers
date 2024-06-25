from setuptools import find_packages, setup

package_name = "kuka_rsi_robot_tests"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KUKA Deutschland GmbH",
    maintainer_email="ravi.rathnam@kuka.com",
    description="This package contains test nodes which connect to a "
    "kuka rsi robot and runs some basic movements",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_movement_test = kuka_rsi_robot_tests.robot_movement_test:main",
        ],
    },
)

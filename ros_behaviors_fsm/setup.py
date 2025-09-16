from setuptools import find_packages, setup

package_name = "ros_behaviors_fsm"

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
    maintainer="Zara, Lily, Zahra",
    maintainer_email="themightyzc@gmail.com, lilywei2023@gmail.com, the.zahra.lari@gmail.com",
    description="Code to make the Neato robot perform several behaviors in a finite state machine.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop = ros_behaviors_fsm.teleop:main",
            "draw_square = ros_behaviors_fsm.draw_square:main",
        ],
    },
)

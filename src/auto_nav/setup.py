from setuptools import find_packages, setup


package_name = "auto_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/mapping.launch.py",
                "launch/navigation.launch.py",
                "launch/exploration.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/exploration.rviz",
                "config/nav2.yaml",
                "config/slam_localization.yaml",
                "config/slam_mapping.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Guido Team",
    maintainer_email="todo@guido.dev",
    description="Autonomous navigation orchestration for the Guido wheelchair.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "auto_nav_command_node = navigation.command_node:main",
            "auto_nav_navigation_node = navigation.navigation_node:main",
        ],
    },
)

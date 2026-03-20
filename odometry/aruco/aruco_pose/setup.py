from setuptools import find_packages, setup

package_name = "aruco_pose"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sverk",
    maintainer_email="petayyyy@gmail.com",
    description="Utilities for generating ArUco marker map files for aruco_map.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "genmap.py = aruco_pose.genmap:main",
        ],
    },
)


from glob import glob
from setuptools import find_packages, setup

package_name = "shapes_turtles"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (f"share/{package_name}/launch", glob(f"launch/launch_*.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rami",
    maintainer_email="rami.sab07@gmail.com",
    description="For drawing primitive shapes in turtlesim.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "draw_shape = shapes_turtles.draw_shape:main",
        ],
    },
)

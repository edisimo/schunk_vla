from glob import glob

from setuptools import setup

package_name = "lwa4p_vla_bridge"

setup(
    name=package_name,
    version="0.3.2",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Eduard Zelenay",
    description="TCP bridge between a LeRobot process and the LWA 4P ROS 2 driver",
    license="MIT",
    entry_points={"console_scripts": [f"bridge = {package_name}.bridge_node:main"]},
)

from glob import glob
from setuptools import find_packages, setup

setup(
    name="warehouse_relay_ideal", version="0.2.0",
    packages=find_packages(include=["ideal_sim", "ideal_sim.*", "relay_learning", "relay_learning.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/warehouse_relay_ideal"]),
        ("share/warehouse_relay_ideal", ["package.xml", "ROS2_GAZEBO.md", "IDEAL_SIM.md"]),
        ("share/warehouse_relay_ideal/launch", glob("launch/*.launch.py")),
        ("share/warehouse_relay_ideal/worlds", glob("worlds/*.sdf")),
        ("share/warehouse_relay_ideal/examples", glob("examples/*.json")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Warehouse relay project team", maintainer_email="team@example.invalid",
    description="Ideal warehouse relay scheduling and map-change experiments",
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": [
        "ideal_scheduler = ideal_sim.ros_node:main",
        "ideal_demo = ideal_sim.cli:main",
    ]},
)

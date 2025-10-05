from setuptools import find_packages, setup
from glob import glob

package_name = "wheatley"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}", ["package.xml"]),
        # Install all launch files
        (f"share/{package_name}/launch", glob("launch/*")),
        # Install all urdf files
        (f"share/{package_name}/urdf", glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dcyoung",
    maintainer_email="dcyoung@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "hello_world_node = wheatley.hello_world_node:main",
            "bot_state_publisher_node = wheatley.bot_state_publisher_node:main",
        ],
    },
)

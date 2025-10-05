from setuptools import find_packages, setup
from glob import glob

PACKAGE_NAME = "wheatley"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        # Install all launch files
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*")),
        # Install all URDF/XACRO files
        (f"share/{PACKAGE_NAME}/urdf", glob("urdf/*")),
        # Install all mesh files
        (f"share/{PACKAGE_NAME}/meshes", glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dcyoung",
    maintainer_email="dcyoung@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "hello_world_node = wheatley.hello_world_node:main",
            "bot_state_publisher_node = wheatley.bot_state_publisher_node:main",
        ],
    },
)

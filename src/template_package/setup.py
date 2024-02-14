import os

from glob import glob
from setuptools import find_packages, setup

package_name = "template_package"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="federico",
    maintainer_email="federico.zappone@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publisher = template_package.publisher_node:main",
            "subscriber = template_package.subscriber_node:main",
            "service = template_package.service_node:main",
            "client = template_package.client_node:main",
        ],
    },
)

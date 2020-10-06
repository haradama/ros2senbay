from setuptools import setup
from setuptools import find_packages

with open("requirements.txt") as requirements:
    install_requirements = requirements.read().splitlines()

setup(
    name="ros2senbay",
    version="0.0.1",
    description="ros2senbay is a ros2 publisher generator that serves senbay-data via ros2 message.",
    author="Masafumi Harada",
    install_requires=install_requirements,
    packages=find_packages(),
    include_package_data=True,
    entry_points={
        "console_scripts": [
            "ros2senbay = ros2senbay.publisherGenerator:main"
        ]
    }
)

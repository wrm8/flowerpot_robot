from setuptools import find_packages
from setuptools import setup

setup(
    name='yolo11_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('yolo11_msgs', 'yolo11_msgs.*')),
)

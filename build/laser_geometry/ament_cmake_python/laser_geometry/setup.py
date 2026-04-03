from setuptools import find_packages
from setuptools import setup

setup(
    name='laser_geometry',
    version='2.11.2',
    packages=find_packages(
        include=('laser_geometry', 'laser_geometry.*')),
)

from setuptools import find_packages
from setuptools import setup

setup(
    name='ugv_interface',
    version='0.0.0',
    packages=find_packages(
        include=('ugv_interface', 'ugv_interface.*')),
)

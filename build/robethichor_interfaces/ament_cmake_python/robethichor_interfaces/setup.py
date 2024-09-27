from setuptools import find_packages
from setuptools import setup

setup(
    name='robethichor_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('robethichor_interfaces', 'robethichor_interfaces.*')),
)

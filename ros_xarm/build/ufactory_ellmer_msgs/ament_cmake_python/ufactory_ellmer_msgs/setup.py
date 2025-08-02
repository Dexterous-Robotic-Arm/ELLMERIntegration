from setuptools import find_packages
from setuptools import setup

setup(
    name='ufactory_ellmer_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('ufactory_ellmer_msgs', 'ufactory_ellmer_msgs.*')),
)

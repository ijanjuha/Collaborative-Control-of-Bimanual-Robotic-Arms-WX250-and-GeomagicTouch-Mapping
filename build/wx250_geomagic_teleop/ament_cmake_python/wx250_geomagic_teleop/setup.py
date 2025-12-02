from setuptools import find_packages
from setuptools import setup

setup(
    name='wx250_geomagic_teleop',
    version='0.0.1',
    packages=find_packages(
        include=('wx250_geomagic_teleop', 'wx250_geomagic_teleop.*')),
)

from setuptools import setup, find_packages
from os import path
working_directory = path.abspath(path.dirname(__file__))

"""
Get started using

> python setup.py sdist bdist_wheel

PUBLIC:
> twine upload dist/*
> pip install robotkinematicscatalogue

TEST:
> python -m twine upload --repository pypi dist/*
> pip install -i https://test.pypi.org/simple/ robotkinematicscatalogue
"""

with open(path.join(working_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name = 'robotkinematicscatalogue',
    version='1.1.0',
    #url=https://github.com/SaltworkerMLU/RobotKinematicsCatalogue
    author='Mathias Lykholt-Ustrup',
    author_email='<saltworkermlu@gmail.com>',
    description='Closed-form Inverse Kinematic Solutions, Forward Kinematics, and Trajectory Generation in one repository',
    long_description=long_description,
    long_description_content_type='text/markdown',
    package=find_packages(),
    install_requires=['scipy', 'numpy', 'sympy', 'matplotlib']
)
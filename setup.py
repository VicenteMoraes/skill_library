from setuptools import setup

setup(
    name='skill_library',
    version='',
    packages=['behaviours', 'launch'],
    url='https://github.com/VicenteMoraes/skill_library',
    license='',
    author='VicenteMoraes',
    author_email='',
    description='Library for implementing Distributed Robotics experiment trials with Docker.',
    install_requires=['asyncio>=3.4.3', 'numpy>=1.24']
)

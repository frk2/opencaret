from setuptools import setup, find_packages

setup(
    name='util',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools','util'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='Utilities ',
    license='MIT'
)
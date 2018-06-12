from setuptools import setup, find_packages

setup(
    name='external',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='External',
    license='MIT',
)
from setuptools import setup, find_packages

setup(
    name='tracker',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'opendbc'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='Object tracker',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_tracker = tracker.obstacle_tracker:main',
        ],
    },
)
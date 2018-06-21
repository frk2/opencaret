from setuptools import setup, find_packages

setup(
    name='radar',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'opendbc'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='Radar controller ',
    license='MIT',
    entry_points={
        'console_scripts': [
            'radar_controller = radar.toyota_radar_controller:main'
        ],
    },
)
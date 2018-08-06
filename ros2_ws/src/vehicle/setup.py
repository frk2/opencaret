from setuptools import setup, find_packages

setup(
    name='vehicle',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='Vehicle Can decoder ',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kia_soul_driver = vehicle.kia_soul_driver:main'
        ],
    },
)
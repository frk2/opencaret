from setuptools import setup, find_packages

setup(
    name='canoc',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='Can transceiver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'transceiver = canoc.transceiver:main',
        ],
    },
)
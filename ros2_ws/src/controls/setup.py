from setuptools import setup, find_packages

setup(
    name='controls',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'opendbc'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='Vehicle longitudinal controller ',
    license='MIT',
    entry_points={
        'console_scripts': [
            'longitudinal_control = controls.longitudinal_control:main',
            'razor_imu = controls.razor_imu:main'
        ],
    },
)
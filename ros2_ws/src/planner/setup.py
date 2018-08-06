from setuptools import setup, find_packages

setup(
    name='planner',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='Motion Planner',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner = planner.longitudinal_planner:main'
        ],
    },
)
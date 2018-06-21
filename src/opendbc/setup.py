from setuptools import setup, find_packages

setup(
    name='opendbc',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={
        'opendbc': ['*.dbc']
    },
    install_requires=['setuptools'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='External',
    license='MIT',
)
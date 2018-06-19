from setuptools import setup, find_packages

setup(
    name='oscc',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={
        'oscc': ['*.dbc']
    },
    install_requires=['setuptools'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='External',
    license='MIT',
)
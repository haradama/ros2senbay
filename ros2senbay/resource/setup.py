from setuptools import setup

package_name = 'ros2senbay'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools'
    ],
    zip_safe=True,
    maintainer='parallels',
    description='ros2senbay is a ros2 publisher generator that serves senbay-data via ros2 message.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'play = ros2senbay.publisher:main',
        ],
    },
)

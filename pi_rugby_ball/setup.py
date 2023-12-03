import os
from glob import glob
from setuptools import setup

package_name = 'pi_rugby_ball'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name + "/launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Andy Blight',
    maintainer_email='a.j.blight@leeds.ac.uk',
    description='Raspberry Pi Rugby Ball top level package for launch files.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    },
)

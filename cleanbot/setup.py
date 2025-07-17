from glob import glob
import os
from setuptools import setup

package_name = 'cleanbot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('params/*')),
        (os.path.join('share', package_name), glob('behaviour_trees/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wasif',
    maintainer_email='mwasifraza10@gmail.com',
    description='Implementation using Opennav Coverage Planning Package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'coverage = cleanbot.coverage:main',
                'gui_coverage = cleanbot.gui_coverage:main',
        ],
    },
)

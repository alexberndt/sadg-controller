from setuptools import setup, find_packages
from glob import glob
import os

def find_files(directory, target_base_path):
    paths_dict = {}
    data_files = []

    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            src_path = os.path.join(path, filename)
            target_path = os.path.join(target_base_path, path)
            if target_path in paths_dict.keys():
                paths_dict[target_path].append(src_path)
            else:
                paths_dict[target_path] = [src_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

package_name = 'sadg_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *find_files('data', os.path.join('share', package_name)),
        *find_files('launch', os.path.join('share', package_name)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Berndt',
    maintainer_email='berndtae@gmail.com',
    description='Implementation of the SADG RHC feedback control scheme to reduce route completion times of delayed agents following MAPF plans.',
    license='Proprietary',
    tests_require=[],
    entry_points={
        'console_scripts': [
        ],
    },
)

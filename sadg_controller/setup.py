from setuptools import setup, find_packages

package_name = 'sadg_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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

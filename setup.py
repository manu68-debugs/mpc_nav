import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mpc_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'waypoints'), glob(os.path.join('waypoints', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
    ],
    install_requires=['setuptools', 'scipy', 'numpy', 'casadi'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='CasADi based MPC tracking and spline generation package.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_controller = mpc_nav.mpc_controller:main',
        ],
    },
)

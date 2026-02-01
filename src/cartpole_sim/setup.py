from setuptools import setup
import os
from glob import glob

package_name = 'cartpole_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/cartpole_sim']),
    ('share/cartpole_sim', ['package.xml']),
    ('share/cartpole_sim/launch', ['launch/sim.launch.py']),
    ('share/cartpole_sim/urdf', glob('urdf/*')),
    ('share/cartpole_sim/config', ['config/controllers.yaml']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gbk08',
    description='cartpole sim',
    license='Apache',
)

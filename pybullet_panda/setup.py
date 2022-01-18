from setuptools import setup
from glob import glob

package_name = 'pybullet_panda'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.*')),
        ('share/' + package_name + '/config', glob('config/*.*')),
        ('lib/' + package_name, [package_name+"/" + el for el in ["trajectory_follower.py"]])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubb',
    maintainer_email='2330834a@student.gla.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "panda=pybullet_panda.panda:main"
        ],
    },
)

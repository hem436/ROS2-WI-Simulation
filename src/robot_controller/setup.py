from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/robot_description.launch.py']),
        ('share/'+ package_name +"/urdf/standard/", glob('urdf/standard/*')),
        ('share/'+ package_name+ "/urdf/sensors/", glob('urdf/sensors/*')),
        ('share/'+ package_name+ "/urdf/lite/", glob('urdf/lite/*')),
        ('share/'+ package_name+ "/meshes/", glob('meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hem436',
    maintainer_email='hemant436268s@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = robot_controller.robot_node:main",
            "draw_circle = robot_controller.draw_circle:main"
        ],
    },
)

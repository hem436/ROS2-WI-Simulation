from setuptools import find_packages, setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name,['detections/detected_objects.log']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hem436',
    maintainer_email='hemant436268s@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "object_detection = object_detection.yolo_detection:main"
        ],
    },
)

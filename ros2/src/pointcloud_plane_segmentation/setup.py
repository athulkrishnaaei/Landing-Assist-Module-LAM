import os
from glob import glob
from setuptools import setup

package_name = 'pointcloud_plane_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    py_modules=[
        'pointcloud_plane_segmentation.plane_segmentation'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='athul',
    maintainer_email='athulkrishnakdgr@gmail.com',
    description='Plane Segmentation Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plane_segmentation = pointcloud_plane_segmentation.plane_segmentation:main'
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'ov9782_trig_sync'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Herschenglime',
    maintainer_email='herschenglime@gmail.com',
    description='Package to trigger ov9782 on receipt of point cloud message',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trig_sync = ov9782_trig_sync.trig_sync:main'
        ],
    },
)

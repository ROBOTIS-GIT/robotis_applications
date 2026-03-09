from setuptools import find_packages, setup

package_name = 'robotis_vuer'

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
    maintainer='ywh',
    maintainer_email='ywh@robotis.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
    },
    entry_points={
        'console_scripts': [
            'vr_publisher_sg2 = robotis_vuer.vr_publisher_sg2:main',
            'vr_publisher_sh5 = robotis_vuer.vr_publisher_sh5:main',
        ],
    },
)

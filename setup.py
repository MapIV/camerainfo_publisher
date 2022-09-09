from setuptools import setup

package_name = 'pe_calibration_publisher'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Perception Engine',
    maintainer_email='contact@perceptionengine.jp',
    description='pe_calibration_publisher',
    license='Apache 2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pe_calibration_publisher = pe_calibration_publisher.pe_calibration_publisher:main',
        ],
    },
)

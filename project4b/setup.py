from setuptools import find_packages, setup
import glob

package_name = 'project4b'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab2004',
    maintainer_email='lab2004@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_drive_simulator = project4b.differential_drive_simulator:main',
            'velocity_translator = project4b.velocity_translator:main',
        ],
    },
)

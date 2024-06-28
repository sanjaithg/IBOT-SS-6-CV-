from setuptools import find_packages, setup

package_name = 'thumb_control'

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
    maintainer='saber',
    maintainer_email='saber@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "thumb_direction_detector = thumb_control.thumb_direction_detector:main",
            "turtle_control = thumb_control.turtle_control:main"
        ],
    },
)

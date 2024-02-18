from setuptools import find_packages, setup

package_name = 'led_control'

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
    maintainer='msr',
    maintainer_email='241abhishek@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_control = led_control.test:main',
            'circle_display = led_control.circle_display:main',
            'image_viewer = led_control.image_viewer:main',
        ],
    },
)

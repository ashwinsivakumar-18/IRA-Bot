from setuptools import setup

package_name = 'teleop_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ashwin',
    maintainer_email='ashwinsivakumar.in@gmail.com',
    description='Simple keyboard teleop',
    license='Apache License 2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = teleop_pkg.teleop_keyboard:main',
        ],
    },
)

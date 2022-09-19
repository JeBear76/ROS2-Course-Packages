from setuptools import setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jebear',
    maintainer_email='jebear@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_transmitter = my_py_pkg.robot_transmitter:main",
            "robot_receiver = my_py_pkg.robot_receiver:main",
            "hardware_status_publisher = my_py_pkg.hardware_status_publisher:main",
            "battery_life_simulator = my_py_pkg.battery_life_simulator:main"
        ],
    },
)

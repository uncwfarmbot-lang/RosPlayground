from setuptools import setup

package_name = 'stepper_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Caddell',
    maintainer_email='james@example.com',
    description='ROS2 Stepper Motor Control Package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stepper_forever = stepper_pkg.stepper_forever:main',
            'stepper_variable = stepper_pkg.stepper_variable:main',
            'stepper_motor = stepper_pkg.stepper_motor:main',
            'keyboard_controller = stepper_pkg.keyboard_controller:main',
        ],
    },
)
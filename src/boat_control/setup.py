from setuptools import setup

package_name = 'boat_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'keyboard_teleop = boat_control.keyboard_teleop:main',
        ],
    },
)

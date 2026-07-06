from setuptools import find_packages, setup

package_name = 'scout_waypoint_navigation'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='levir',
    maintainer_email='levir@estudante.ufscar.br',
    description=(
        'Navegação autônoma do Scout Mini com '
        'FAST-LIO, waypoints e desvio de obstáculos.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_controller = '
            'scout_waypoint_navigation.waypoint_controller:main',

            'obstacle_detector = '
            'scout_waypoint_navigation.obstacle_detector:main',

            'autonomous_navigator = '
            'scout_waypoint_navigation.autonomous_navigator:main',

            'mission_navigator = '
            'scout_waypoint_navigation.mission_navigator:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'project_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/launch', ['launch/p_mission_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea-vbox',
    maintainer_email='andrea-vbox@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mission = project_pkg.mission:main",
            "flask_mission = project_pkg.mission_flask:main"
            ],
    },
)

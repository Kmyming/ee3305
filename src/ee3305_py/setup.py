from setuptools import find_packages, setup

package_name = 'ee3305_py'

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
    maintainer='Lai Yan Kai',
    maintainer_email='lai.yankai@gmail.com',
    description='https://www.youtube.com/watch?v=dQw4w9WgXcQ',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior = ee3305_py.behavior:main',
            'planner = ee3305_py.planner:main',
            'controller = ee3305_py.controller:main',
            'evaluator = ee3305_py.evaluator:main'
        ],
    },
)

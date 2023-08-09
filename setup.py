from setuptools import find_packages, setup

package_name = 'rgb_points'

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
    maintainer='iviraldo',
    maintainer_email='isabelleviraldo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cp_v1 = rgb_points.initial_attempt:main',
            'cp_new_math = rgb_points.fixed_math:main',
            'cp_center = rgb_points.color_objects_using_center:main',
            'cp_few_colors = rgb_points.garbage_fire:main'
        ],
    },
)

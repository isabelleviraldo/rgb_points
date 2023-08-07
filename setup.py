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
            'read_pcl = rgb_points.reading_point_cloud:main',
            'read_img = rgb_points.reading_image:main',
            'read_obj = rgb_points.reading_clusters:main',
            'test_math = rgb_points.math:main',
            'color_points_v1 = rgb_points.initial_attempt:main',
            'color_points_new_math = rgb_points.fixed_math:main',
            'color_points_center = rgb_points.color_objects_using_center:main',
            'garbage_fire_testing = rgb_points.garbage_fire:main'
        ],
    },
)

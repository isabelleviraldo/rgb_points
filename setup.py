from setuptools import setup

package_name = 'rgb_images'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isabelleviraldo',
    maintainer_email='isabelleviraldo@gmail.com',
    description='several different attempts to try and publish a point cloud with colored points based off of the image published by the zed camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_pcl = rgb_images.reading_point_cloud:main'
            'read_img = rgb_images.reading_image:main'
            'read_obj = rgb_images.reading_clusters:main'
            'test_math = rgb_images.math:main'
            'color_points_v1 = rgb_images.initial_attempt:main'
            'color_points_new_math = rgb_images.fixed_math:main'
            'color_points_center = rgb_images.color_objects_using_center:main'
            'garbage_fire_testing = rgb_images.garbage_fire:main'
        ],
    },
)

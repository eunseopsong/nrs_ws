from setuptools import setup
package_name = 'urp_cvt_sampling'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cvt_sampler.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='CVT (k-means) waypoint sampler node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cvt_sampler_node = urp_cvt_sampling.cvt_sampler_node:main',
            'path_planner_node = urp_cvt_sampling.path_planner_node:main',
        ],
    },
)

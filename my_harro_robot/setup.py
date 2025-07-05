from glob import glob
from setuptools import find_packages, setup

package_name = 'my_harro_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/trees', glob('trees/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/textures', glob('textures/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/tree_textures', glob('tree_textures/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phani',
    maintainer_email='phani@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_detection_node = my_harro_robot.pose_detection:main',
            'move_bot = my_harro_robot.move_bot:main',
            'move_me = my_harro_robot.move_me:teleop_node',
        ],
    },
)

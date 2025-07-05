#!/usr/bin/env python3
import os
import glob
import xacro
import shutil
import random
import tempfile

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg = get_package_share_directory('my_harro_robot')

raw_models = os.path.join(pkg, 'models')
raw_textures = os.path.join(pkg, 'textures')

tmp_base = os.path.join(tempfile.gettempdir(), 'harro_resources')
models_tmp = os.path.join(tmp_base, 'models')
textures_tmp = os.path.join(tmp_base, 'textures')
sdf_tmp = os.path.join(tempfile.gettempdir(), 'harro_sdfs')


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo.launch.py')
        )
    ))

    xacro_file = os.path.join(pkg, 'urdf', 'harro.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    ))
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    ))
    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic',  'robot_description',
            '-entity', 'tech_robot',
            '-x',      '0.0',
            '-y',      '0.0',
            '-z',      '0.1',
        ],
        output='screen',
    ))

    for d in (models_tmp, sdf_tmp):
        if os.path.isdir(d):
            shutil.rmtree(d)
        os.makedirs(d)
    # textures: just remove old then copytree
    if os.path.isdir(textures_tmp):
        shutil.rmtree(textures_tmp)
    if os.path.isdir(raw_textures):
        shutil.copytree(raw_textures, textures_tmp)

    # 4) Clean each .dae → models_tmp, wrap into wrapper SDF in sdf_tmp
    dae_list = glob.glob(os.path.join(raw_models, '*.dae'))
    for idx, dae in enumerate(dae_list, start=1):
        clean_dae = os.path.join(models_tmp, os.path.basename(dae))
        with open(dae, 'r', encoding='utf-8') as fin, \
             open(clean_dae, 'w', encoding='utf-8') as fout:
            for line in fin:
                if line.lstrip().startswith('<?xml'):
                    continue
                fout.write(line)

        wrapper = os.path.join(sdf_tmp, f'model_{idx}.sdf')
        uri     = f"file://{clean_dae}"
        scale   = "0.2 0.2 0.2"

        sdf = f"""<sdf version="1.6">
          <model name="model_{idx}">
            <static>false</static>
            <link name="link">
              <visual name="visual">
                    <geometry>
                            <mesh>
                                    <uri>{uri}</uri>
                                    <scale>{scale}</scale>
                            </mesh>
                    </geometry>
              </visual>
              <collision name="collision">
                    <geometry>
                            <mesh>
                                    <uri>{uri}</uri>
                                    <scale>{scale}</scale>
                            </mesh>
                    </geometry>
              </collision>
            </link>
          </model>
        </sdf>"""
        with open(wrapper, 'w', encoding='utf-8') as f:
            f.write(sdf)

    # 5) After 5s, spawn all wrappers + add overhead light
    spawn_actions = []
    for idx in range(1, len(dae_list)+1):
        wrapper = os.path.join(sdf_tmp, f'model_{idx}.sdf')
        x, y = random.uniform(-5,5), random.uniform(-5,5)
        spawn_actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_model_{idx}',
            output='screen',
            arguments=[
                '-entity', f'model_{idx}',
                '-file',   wrapper,
                '-x',      str(x),
                '-y',      str(y),
                '-z',      '0.0',
            ],
        ))

    # build a light SDF file
    light_sdf = os.path.join(tempfile.gettempdir(), 'top_light.sdf')
    light_content = """<sdf version="1.6">
      <light name="top_light" type="point">
        <pose>0 0 5 0 0 0</pose>
        <diffuse>1 1 1 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <attenuation>
          <range>30</range>
          <constant>0.3</constant>
          <linear>0.01</linear>
          <quadratic>0.001</quadratic>
        </attenuation>
        <cast_shadows>true</cast_shadows>
      </light>
    </sdf>
    """
    with open(light_sdf, 'w', encoding='utf-8') as f:
        f.write(light_content)

    spawn_actions.append(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_top_light',
        output='screen',
        arguments=[
            '-entity', 'top_light',
            '-file',   light_sdf,
            '-x',      '0.0',
            '-y',      '0.0',
            '-z',      '0.0',
        ],
    ))

    ld.add_action(TimerAction(period=5.0, actions=spawn_actions))
    return ld

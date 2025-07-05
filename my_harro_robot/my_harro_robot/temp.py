#!/usr/bin/env python3

import os
import xacro
import shutil
import tempfile

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

# === SETUP PATHS ===
pkg = get_package_share_directory('my_harro_robot')
raw_models = os.path.join(pkg, 'models')
raw_textures = os.path.join(pkg, 'textures')

tmp_base = os.path.join(tempfile.gettempdir(), 'harro_resources')
models_tmp = os.path.join(tmp_base, 'models')
textures_tmp = os.path.join(tmp_base, 'textures')
sdf_tmp = os.path.join(tempfile.gettempdir(), 'harro_sdfs')
light_sdf = os.path.join(tempfile.gettempdir(), 'top_light.sdf')

# === LOAD ROBOT DESCRIPTION ===
xacro_file = os.path.join(pkg, 'urdf', 'harro.xacro')
robot_desc = xacro.process_file(xacro_file).toxml()

# === CLEAN FOLDERS ===
for d in (models_tmp, sdf_tmp):
    if os.path.exists(d):
        shutil.rmtree(d)
    os.makedirs(d)

if os.path.exists(textures_tmp):
    shutil.rmtree(textures_tmp)
shutil.copytree(raw_textures, textures_tmp)

# === PROCESS MODELS ===
dae_files = [f for f in os.listdir(raw_models) if f.endswith('.dae')]
dae_list = [os.path.join(raw_models, file) for file in dae_files]
model_names = []

for fname in dae_list:
    name = os.path.basename(fname)
    file_name = name.split(".")[0]
    model_names.append(file_name)

    src_dae = fname
    dst_dae = os.path.join(models_tmp, os.path.basename(fname))

    # Clean XML header
    with open(src_dae, 'r', encoding='utf-8') as fin, open(dst_dae, 'w', encoding='utf-8') as fout:
        for line in fin:
            if not line.strip().startswith('<?xml'):
                fout.write(line)

    # SDF wrapper
    uri = f"file://{dst_dae}"
    scale = "0.3 0.3 0.3"
    sdf_content = f"""<sdf version="1.6">
      <model name="{file_name}">
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
    </sdf>
    """
    with open(os.path.join(sdf_tmp, f"{file_name}.sdf"), 'w', encoding='utf-8') as f:
        f.write(sdf_content)

# === FIXED POSITIONS ===
model_positions = {
    "man_standing": (0.0, 2.0),
    "boy_girl": (0.0, -2.0),
    "girl_with_bag": (2.0, 0.0),
    "man_sofa_sitting": (-2.0, 0.0),
    "boy_on_table": (2.0, 2.0),
}

# === SPAWN ACTIONS ===
spawn_actions = []
for name in model_names:
    wrapper = os.path.join(sdf_tmp, f"{name}.sdf")
    x, y = model_positions.get(name, (0.0, 0.0))
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{name}',
        output='screen',
        arguments=[
            '-entity', name,
            '-file', wrapper,
            '-x', str(x),
            '-y', str(y),
            '-z', '0.0'
        ],
    )
    spawn_actions.append(spawn)

# === SPAWN LIGHT ===
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

spawn_light = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    name='spawn_top_light',
    output='screen',
    arguments=[
        '-entity', 'top_light',
        '-file', light_sdf,
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.0',
    ]
)
spawn_actions.append(spawn_light)

# === FINAL LAUNCH FUNCTION ===
def generate_launch_description():
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo.launch.py')
        )
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        name='robot_state_publisher'
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    urdf_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tech_robot',
        output='screen',
        arguments=[
            '-entity', 'tech_robot',
            '-topic', 'robot_description'
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ]
    )

    delayed_spawns = TimerAction(
        period=5.0,
        actions=spawn_actions
    )

    return LaunchDescription([
        launch_gazebo,
        robot_state_pub,
        joint_state_pub,
        urdf_spawn,
        delayed_spawns
    ])

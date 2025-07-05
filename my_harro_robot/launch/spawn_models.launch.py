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
raw_trees = os.path.join(pkg, 'trees')
raw_tree_textures = os.path.join(pkg, 'tree_textures')

tmp_base = os.path.join(tempfile.gettempdir(), 'harro_resources')

models_tmp = os.path.join(tmp_base, 'models')
textures_tmp = os.path.join(tmp_base, 'textures')
trees_tmp = os.path.join(tmp_base, 'trees')
tree_textures_tmp = os.path.join(tmp_base, 'tree_textures')

sdf_tmp = os.path.join(tempfile.gettempdir(), 'harro_sdfs')
light_sdf = os.path.join(tempfile.gettempdir(), 'top_light.sdf')

# === LOAD ROBOT DESCRIPTION ===
xacro_file = os.path.join(pkg, 'urdf', 'harro.xacro')
robot_desc = xacro.process_file(xacro_file).toxml()

# === CLEAN FOLDERS ===
for d in (models_tmp, sdf_tmp, trees_tmp):
    if os.path.exists(d):
        shutil.rmtree(d)
    os.makedirs(d)

if os.path.exists(textures_tmp):
    shutil.rmtree(textures_tmp)
if os.path.exists(tree_textures_tmp):
    shutil.rmtree(tree_textures_tmp)

shutil.copytree(raw_textures, textures_tmp)
shutil.copytree(raw_tree_textures, tree_textures_tmp)

# === PROCESS MODELS ===
model_files = [f for f in os.listdir(raw_models) if f.endswith('.dae')]
model_list = [os.path.join(raw_models, file) for file in model_files]
tree_files = [f for f in os.listdir(raw_trees) if f.endswith('.dae')]
tree_list = [os.path.join(raw_trees, file) for file in tree_files]

model_names = []
tree_names = []

for fname in model_list:
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
        <static>true</static>
        <link name="link">
          <visual name="visual">
            <geometry>
              <mesh>
                <uri>{uri}</uri>
                <scale>{scale}</scale>
              </mesh>
            </geometry>
          </visual>
        </link>
      </model>
    </sdf>
    """
    with open(os.path.join(sdf_tmp, f"{file_name}.sdf"), 'w', encoding='utf-8') as f:
        f.write(sdf_content)

for fname in tree_list:
    name = os.path.basename(fname)
    file_name = name.split(".")[0]
    tree_names.append(file_name)

    src_dae = fname
    dst_dae = os.path.join(trees_tmp, os.path.basename(fname))

    # Clean XML header
    with open(src_dae, 'r', encoding='utf-8') as fin, open(dst_dae, 'w', encoding='utf-8') as fout:
        for line in fin:
            if not line.strip().startswith('<?xml'):
                fout.write(line)

    # SDF wrapper
    uri = f"file://{dst_dae}"
    scale = "0.15 0.15 0.15"
    sdf_content = f"""<sdf version="1.6">
      <model name="{file_name}">
        <static>true</static>
        <link name="link">
          <visual name="visual">
            <geometry>
              <mesh>
                <uri>{uri}</uri>
                <scale>{scale}</scale>
              </mesh>
            </geometry>
          </visual>
        </link>
      </model>
    </sdf>
    """
    with open(os.path.join(sdf_tmp, f"{file_name}.sdf"), 'w', encoding='utf-8') as f:
        f.write(sdf_content)

# === FIXED POSITIONS ===
model_positions = {
    "man_standing": [(-2.0, -2.0), (0.0, 0.0, 2.355)],
    "boy_girl": [(2.0, 2.0), (0.0, 0.0, -0.785)],
    "girl_with_bag": [(2.0, 0.0), (0.0, 0.0, -1.57)],
    "man_sofa_sitting": [(-2.0, 0.0), (0.0, 0.0, 1.57)],
    "boy_on_table": [(0.0, -2.0), (0.0, 0.0, 3.14)],
    "boy_icecream": [(0.0, 2.0), (0.0, 0.0, 0.0)]
}

tree_positions = {
    "tree_1": [(2.0, -2.0), (0.0, 0.0, 0.785)],
    "tree_2": [(-2.0, 2.0), (0.0, 0.0, 1.57)]
}

# === SPAWN ACTIONS ===
spawn_actions = []
for m_name in model_names:
    wrapper = os.path.join(sdf_tmp, f"{m_name}.sdf")
    (x, y), (R, P, Y) = model_positions.get(m_name, ((0.0, 0.0), (0.0, 0.0, 0.0)))
    model_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{m_name}',
        output='screen',
        arguments=[
            '-entity', m_name,
            '-file', wrapper,
            '-x', str(x),
            '-y', str(y),
            '-z', '0.0',
            '-R', str(R),
            '-P', str(P),
            '-Y', str(Y)
        ],
    )
    spawn_actions.append(model_spawn)

for t_name in tree_names:
    wrapper = os.path.join(sdf_tmp, f"{t_name}.sdf")
    (x, y), (R, P, Y) = tree_positions.get(t_name, ((0.0, 0.0), (0.0, 0.0, 0.0)))
    tree_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{t_name}',
        output='screen',
        arguments=[
            '-entity', t_name,
            '-file', wrapper,
            '-x', str(x),
            '-y', str(y),
            '-z', '0.0',
            '-R', str(R),
            '-P', str(P),
            '-Y', str(Y)
        ]
    )
    spawn_actions.append(tree_spawn)

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

    delayed_spawns = TimerAction(
        period=5.0,
        actions=spawn_actions
    )

    return LaunchDescription([
        launch_gazebo,
        delayed_spawns
    ])

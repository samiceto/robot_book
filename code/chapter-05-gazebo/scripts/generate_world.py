#!/usr/bin/env python3
"""
Generate Gazebo world with randomized obstacles for humanoid navigation testing.

Usage:
    python3 generate_world.py [num_obstacles]

Example:
    python3 generate_world.py 10
"""

import random
import sys
import os


def generate_world(num_obstacles=8, output_file='worlds/obstacle_world.sdf'):
    """
    Generate a Gazebo SDF world file with random obstacles.

    Args:
        num_obstacles: Number of random obstacles to place (default: 8)
        output_file: Output file path (default: worlds/obstacle_world.sdf)
    """
    sdf = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="obstacle_world">

    <!-- Physics Configuration (DART for humanoid stability) -->
    <physics name="dart_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <dart>
        <solver>
          <solver_type>pgs</solver_type>
          <constraint_cfm>0.00001</constraint_cfm>
          <constraint_erp>0.2</constraint_erp>
        </solver>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>

    <!-- Directional Lighting (Sun) -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.5 0.5 0.5 1.0</specular>
      <direction>-0.5 0.1 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Ground Plane (10m x 10m) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>1e7</kp>
                <kd>1e4</kd>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

"""

    # Generate random obstacles (boxes with varying heights)
    random.seed(42)  # Reproducible random placement
    for i in range(num_obstacles):
        x = random.uniform(-4.0, 4.0)
        y = random.uniform(-4.0, 4.0)
        height = random.uniform(0.5, 2.0)
        width = random.uniform(0.3, 0.6)
        depth = random.uniform(0.3, 0.6)

        # Random color
        r = random.uniform(0.3, 0.9)
        g = random.uniform(0.3, 0.9)
        b = random.uniform(0.3, 0.9)

        sdf += f"""    <!-- Obstacle {i} -->
    <model name="obstacle_{i}">
      <pose>{x:.3f} {y:.3f} {height/2:.3f} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{width:.3f} {depth:.3f} {height:.3f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{width:.3f} {depth:.3f} {height:.3f}</size>
            </box>
          </geometry>
          <material>
            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

"""

    sdf += """  </world>
</sdf>
"""

    return sdf


def main():
    """Main function to generate world file."""
    num_obstacles = int(sys.argv[1]) if len(sys.argv) > 1 else 8

    # Create worlds directory if it doesn't exist
    os.makedirs('worlds', exist_ok=True)

    # Generate world SDF
    world_sdf = generate_world(num_obstacles)

    # Write to file
    output_file = 'worlds/obstacle_world.sdf'
    with open(output_file, 'w') as f:
        f.write(world_sdf)

    print(f"✓ Generated world with {num_obstacles} obstacles")
    print(f"✓ Output file: {output_file}")
    print(f"\nLaunch with:")
    print(f"  gz sim {output_file}")


if __name__ == '__main__':
    main()

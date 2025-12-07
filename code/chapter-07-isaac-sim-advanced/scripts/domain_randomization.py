#!/usr/bin/env python3
"""
Domain Randomization Examples for Isaac Sim.

This script demonstrates both visual and physics domain randomization techniques
for sim-to-real transfer. Randomizes:

Visual Parameters:
- Textures and materials
- Object colors
- Lighting intensity and color temperature

Physics Parameters:
- Mass (±20% variance)
- Friction coefficients (0.3-0.9)
- Restitution (0.0-0.8)
- Damping (0.1-10.0)

Usage:
    cd ~/.local/share/ov/pkg/isaac-sim-4.2.0

    # Run full randomization demo
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/domain_randomization.py

    # Visual randomization only
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/domain_randomization.py --visual-only

    # Physics randomization only
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/domain_randomization.py --physics-only

    # Run 100 randomization episodes
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/domain_randomization.py --num-episodes 100
"""

import argparse
import random
import time
from omni.isaac.kit import SimulationApp

parser = argparse.ArgumentParser(description="Domain Randomization Demo")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
parser.add_argument("--num-episodes", type=int, default=50, help="Number of randomization episodes")
parser.add_argument("--visual-only", action="store_true", help="Only randomize visual parameters")
parser.add_argument("--physics-only", action="store_true", help="Only randomize physics parameters")
args, unknown = parser.parse_known_args()

# Launch Isaac Sim
CONFIG = {"headless": args.headless, "width": 1280, "height": 720}
simulation_app = SimulationApp(CONFIG)

import omni
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
import omni.isaac.core.utils.prims as prim_utils
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, Sdf
import carb


def create_simple_scene(world: World):
    """
    Create a simple scene with ground, lighting, and objects to randomize.

    Args:
        world: Isaac Sim World instance

    Returns:
        List of object prim paths
    """
    # Ground plane
    world.scene.add_default_ground_plane()

    # Dome light (for realistic rendering)
    create_prim(
        "/World/DomeLight",
        "DomeLight",
        attributes={"inputs:intensity": 1000.0, "inputs:color": (1.0, 1.0, 1.0)}
    )

    # Create objects to randomize
    object_prims = []

    # Cube
    cube_path = "/World/Cube"
    create_prim(
        cube_path,
        "Cube",
        attributes={"size": 0.5}
    )
    cube_prim = prim_utils.get_prim_at_path(cube_path)
    UsdGeom.Xformable(cube_prim).AddTranslateOp().Set(Gf.Vec3d(-1.0, 0, 0.25))

    # Add physics
    UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
    mass_api.CreateMassAttr(1.0)

    object_prims.append(cube_path)

    # Cylinder
    cylinder_path = "/World/Cylinder"
    create_prim(
        cylinder_path,
        "Cylinder",
        attributes={"radius": 0.25, "height": 0.5}
    )
    cylinder_prim = prim_utils.get_prim_at_path(cylinder_path)
    UsdGeom.Xformable(cylinder_prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.25))

    # Add physics
    UsdPhysics.RigidBodyAPI.Apply(cylinder_prim)
    UsdPhysics.CollisionAPI.Apply(cylinder_prim)
    mass_api = UsdPhysics.MassAPI.Apply(cylinder_prim)
    mass_api.CreateMassAttr(1.0)

    object_prims.append(cylinder_path)

    # Sphere
    sphere_path = "/World/Sphere"
    create_prim(
        sphere_path,
        "Sphere",
        attributes={"radius": 0.25}
    )
    sphere_prim = prim_utils.get_prim_at_path(sphere_path)
    UsdGeom.Xformable(sphere_prim).AddTranslateOp().Set(Gf.Vec3d(1.0, 0, 0.25))

    # Add physics
    UsdPhysics.RigidBodyAPI.Apply(sphere_prim)
    UsdPhysics.CollisionAPI.Apply(sphere_prim)
    mass_api = UsdPhysics.MassAPI.Apply(sphere_prim)
    mass_api.CreateMassAttr(1.0)

    object_prims.append(sphere_path)

    carb.log_info(f"Created scene with {len(object_prims)} objects")
    return object_prims


def randomize_color(prim_path: str):
    """
    Randomize the color of a prim.

    Args:
        prim_path: USD path to prim
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    # Create or get material
    material_path = f"{prim_path}/Material"
    material = UsdShade.Material.Define(stage, material_path)

    # Create shader
    shader_path = f"{material_path}/Shader"
    shader = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")

    # Random color (avoid very dark colors)
    r = random.uniform(0.2, 1.0)
    g = random.uniform(0.2, 1.0)
    b = random.uniform(0.2, 1.0)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(r, g, b))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(random.uniform(0.1, 0.9))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(random.uniform(0.0, 0.3))

    # Create material output
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    # Bind material to prim
    UsdShade.MaterialBindingAPI(prim).Bind(material)

    carb.log_info(f"  Color randomized: R={r:.2f}, G={g:.2f}, B={b:.2f}")


def randomize_lighting(light_prim_path: str = "/World/DomeLight"):
    """
    Randomize lighting intensity and color temperature.

    Args:
        light_prim_path: USD path to light prim
    """
    stage = omni.usd.get_context().get_stage()
    light_prim = stage.GetPrimAtPath(light_prim_path)

    if not light_prim.IsValid():
        carb.log_warn(f"Light prim not found: {light_prim_path}")
        return

    # Random intensity (500-2000 lux)
    intensity = random.uniform(500, 2000)
    light_prim.GetAttribute("inputs:intensity").Set(intensity)

    # Random color temperature (2700K warm to 6500K cool)
    temperature = random.uniform(2700, 6500)

    # Convert color temperature to RGB (simplified Planckian locus approximation)
    if temperature <= 6500:
        r = 1.0
        g = temperature / 6500.0
        b = 0.5 + 0.5 * (temperature / 6500.0)
    else:
        r = 1.0
        g = 1.0
        b = 1.0

    light_prim.GetAttribute("inputs:color").Set(Gf.Vec3f(r, g, b))

    carb.log_info(f"  Lighting randomized: Intensity={intensity:.0f}, Temp={temperature:.0f}K")


def randomize_mass(prim_path: str, variance: float = 0.2):
    """
    Randomize mass of a rigid body by ±variance.

    Args:
        prim_path: USD path to rigid body
        variance: Fractional variance (0.2 = ±20%)
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    mass_api = UsdPhysics.MassAPI(prim)
    if not mass_api:
        carb.log_warn(f"No MassAPI on prim: {prim_path}")
        return

    # Get original mass
    original_mass = mass_api.GetMassAttr().Get()
    if original_mass is None:
        original_mass = 1.0

    # Randomize
    scale = random.uniform(1 - variance, 1 + variance)
    new_mass = original_mass * scale
    mass_api.GetMassAttr().Set(new_mass)

    carb.log_info(f"  Mass randomized: {original_mass:.2f} kg → {new_mass:.2f} kg (scale: {scale:.2f})")


def randomize_friction(prim_path: str):
    """
    Randomize friction coefficient (0.3-0.9 for common materials).

    Args:
        prim_path: USD path to collision prim
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    # Create PhysicsMaterialAPI if not exists
    if not prim.HasAPI(UsdPhysics.MaterialAPI):
        material_api = UsdPhysics.MaterialAPI.Apply(prim)
    else:
        material_api = UsdPhysics.MaterialAPI(prim)

    # Random friction (wood: 0.3, rubber: 0.9)
    static_friction = random.uniform(0.3, 0.9)
    dynamic_friction = static_friction * random.uniform(0.8, 1.0)  # Dynamic < static

    material_api.CreateStaticFrictionAttr(static_friction)
    material_api.CreateDynamicFrictionAttr(dynamic_friction)

    carb.log_info(f"  Friction randomized: Static={static_friction:.2f}, Dynamic={dynamic_friction:.2f}")


def randomize_restitution(prim_path: str):
    """
    Randomize restitution (bounciness) coefficient (0.0-0.8).

    Args:
        prim_path: USD path to collision prim
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    # Create PhysicsMaterialAPI if not exists
    if not prim.HasAPI(UsdPhysics.MaterialAPI):
        material_api = UsdPhysics.MaterialAPI.Apply(prim)
    else:
        material_api = UsdPhysics.MaterialAPI(prim)

    # Random restitution (rubber ball: 0.8, wood: 0.1)
    restitution = random.uniform(0.0, 0.8)
    material_api.CreateRestitutionAttr(restitution)

    carb.log_info(f"  Restitution randomized: {restitution:.2f}")


def randomize_scene(object_prims: list, visual: bool = True, physics: bool = True):
    """
    Randomize all parameters in the scene.

    Args:
        object_prims: List of object prim paths
        visual: Enable visual randomization
        physics: Enable physics randomization
    """
    carb.log_info("=" * 70)
    carb.log_info("RANDOMIZING SCENE")
    carb.log_info("=" * 70)

    if visual:
        carb.log_info("Visual Randomization:")
        for prim_path in object_prims:
            randomize_color(prim_path)
        randomize_lighting()

    if physics:
        carb.log_info("Physics Randomization:")
        for prim_path in object_prims:
            randomize_mass(prim_path, variance=0.2)
            randomize_friction(prim_path)
            randomize_restitution(prim_path)

    carb.log_info("=" * 70)


def main():
    """Main function."""

    # Create world
    world = World(stage_units_in_meters=1.0)

    # Create scene
    object_prims = create_simple_scene(world)

    # Reset world to initialize physics
    world.reset()

    carb.log_info("")
    carb.log_info("=" * 70)
    carb.log_info("DOMAIN RANDOMIZATION DEMO")
    carb.log_info("=" * 70)
    carb.log_info(f"Mode: ", end="")
    if args.visual_only:
        carb.log_info("Visual Only")
    elif args.physics_only:
        carb.log_info("Physics Only")
    else:
        carb.log_info("Full (Visual + Physics)")
    carb.log_info(f"Episodes: {args.num_episodes}")
    carb.log_info(f"Objects: {len(object_prims)}")
    carb.log_info("=" * 70)
    carb.log_info("")

    # Determine randomization modes
    visual = not args.physics_only
    physics = not args.visual_only

    # Randomization loop
    episode = 0
    frames_per_episode = 120  # 2 seconds at 60 FPS
    frame_count = 0

    while simulation_app.is_running() and episode < args.num_episodes:
        # Step simulation
        world.step(render=True)
        frame_count += 1

        # Randomize every episode
        if frame_count >= frames_per_episode:
            episode += 1
            frame_count = 0

            carb.log_info(f"\nEpisode {episode}/{args.num_episodes}")
            randomize_scene(object_prims, visual=visual, physics=physics)

            # Reset world to apply new parameters
            world.reset()

    carb.log_info("")
    carb.log_info("=" * 70)
    carb.log_info("DEMO COMPLETE")
    carb.log_info("=" * 70)
    carb.log_info(f"Total Episodes: {episode}")
    carb.log_info("=" * 70)

    simulation_app.close()


if __name__ == "__main__":
    main()

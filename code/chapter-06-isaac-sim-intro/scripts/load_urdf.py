#!/usr/bin/env python3
"""
Load URDF into Isaac Sim and configure PhysX parameters.

This script demonstrates:
1. Launching Isaac Sim via SimulationApp
2. Importing URDF as USD
3. Configuring PhysX articulation parameters
4. Running physics simulation

Usage:
    cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/load_urdf.py
"""

import argparse
import sys
from omni.isaac.kit import SimulationApp

# Parse arguments before launching Isaac Sim
parser = argparse.ArgumentParser()
parser.add_argument(
    "--urdf",
    type=str,
    default="~/ros2_ws/src/robot-book-code/chapter-04-urdf-sdf/urdf/humanoid_12dof.urdf",
    help="Path to URDF file"
)
parser.add_argument("--headless", action="store_true", help="Run without GUI")
args, unknown = parser.parse_known_args()

# Launch Isaac Sim
CONFIG = {"headless": args.headless}
simulation_app = SimulationApp(CONFIG)

import os
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
import omni.isaac.core.utils.prims as prim_utils
import carb

# Enable URDF importer extension
enable_extension("omni.isaac.urdf")


def load_urdf_to_stage(urdf_path: str, robot_prim_path: str = "/World/Humanoid"):
    """
    Load URDF file into USD stage using Isaac Sim URDF importer.

    Args:
        urdf_path: Absolute path to URDF file
        robot_prim_path: USD path for root prim

    Returns:
        Root prim path of imported robot
    """
    from omni.isaac.urdf import _urdf

    # Expand path
    urdf_path = os.path.expanduser(urdf_path)

    if not os.path.exists(urdf_path):
        carb.log_error(f"URDF file not found: {urdf_path}")
        return None

    carb.log_info(f"Loading URDF: {urdf_path}")

    # Get URDF interface
    urdf_interface = _urdf.acquire_urdf_interface()

    # Import config
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False  # Allow humanoid to move freely
    import_config.distance_scale = 1.0
    import_config.density = 0.0  # Use URDF masses, not density
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 5000.0  # PD controller stiffness
    import_config.default_position_drive_damping = 500.0  # PD damping

    # Parse URDF
    success, robot_path = urdf_interface.parse_urdf(
        urdf_path, robot_prim_path, import_config
    )

    if not success:
        carb.log_error("Failed to import URDF")
        return None

    carb.log_info(f"URDF imported successfully to: {robot_path}")
    return robot_path


def configure_physics(robot_prim_path: str):
    """
    Configure PhysX parameters for humanoid robot.

    Args:
        robot_prim_path: USD path to robot root prim
    """
    prim = prim_utils.get_prim_at_path(robot_prim_path)
    if not prim:
        carb.log_error(f"Prim not found: {robot_prim_path}")
        return

    # Apply Physics Articulation API
    if not prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
        physx_articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    else:
        physx_articulation_api = PhysxSchema.PhysxArticulationAPI(prim)

    # Configure articulation parameters
    physx_articulation_api.CreateSolverPositionIterationCountAttr(8)  # Stability
    physx_articulation_api.CreateSolverVelocityIterationCountAttr(4)
    physx_articulation_api.CreateSleepThresholdAttr(0.005)
    physx_articulation_api.CreateStabilizationThresholdAttr(0.001)
    physx_articulation_api.CreateEnabledSelfCollisionsAttr(False)  # Performance

    carb.log_info(f"PhysX articulation configured for: {robot_prim_path}")


def main():
    """Main function to load URDF and run simulation."""

    # Create world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Add lighting
    from omni.isaac.core.utils.prims import create_prim
    create_prim(
        "/World/DomeLight",
        "DomeLight",
        attributes={
            "inputs:intensity": 1000.0,
            "inputs:texture:file": "omniverse://localhost/NVIDIA/Assets/Skies/Indoor/ZetoCG_com_WarehouseInterior2b.hdr"
        }
    )

    # Load URDF
    robot_path = load_urdf_to_stage(args.urdf, "/World/Humanoid")
    if not robot_path:
        carb.log_error("Failed to load URDF. Exiting.")
        simulation_app.close()
        sys.exit(1)

    # Position robot above ground
    robot_prim = prim_utils.get_prim_at_path(robot_path)
    UsdGeom.Xformable(robot_prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.0))

    # Configure physics
    configure_physics(robot_path)

    # Reset world
    world.reset()

    carb.log_info("="*60)
    carb.log_info("Isaac Sim URDF Loader - Ready")
    carb.log_info("="*60)
    carb.log_info("Press PLAY (Space) in Isaac Sim to start simulation")
    carb.log_info("Press STOP to reset")
    carb.log_info("="*60)

    # Simulation loop
    frame_count = 0
    while simulation_app.is_running():
        world.step(render=True)
        frame_count += 1

        # Print stats every 100 frames
        if frame_count % 100 == 0:
            carb.log_info(f"Simulation running... Frame: {frame_count}")

    # Cleanup
    simulation_app.close()


if __name__ == "__main__":
    main()

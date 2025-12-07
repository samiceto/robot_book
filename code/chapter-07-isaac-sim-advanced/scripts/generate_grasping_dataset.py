#!/usr/bin/env python3
"""
Generate 1000-image grasping dataset for humanoid manipulation training.

This script is the Hands-On Lab for Chapter 7. It generates a synthetic dataset
with randomized object positions, robot configurations, and annotations suitable
for training vision-language-action (VLA) models.

Outputs:
- 1000 RGB images (640×480)
- 1000 depth maps (16-bit PNG)
- 1000 semantic segmentation masks
- Bounding boxes in COCO format
- Metadata JSON with object classes

Usage:
    cd ~/.local/share/ov/pkg/isaac-sim-4.2.0

    # Generate full 1000-image dataset
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/generate_grasping_dataset.py \
      --num-images 1000 \
      --output-dir ~/datasets/grasping_dataset

    # Quick test with 10 images
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/generate_grasping_dataset.py \
      --num-images 10

    # Headless mode for maximum throughput
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/generate_grasping_dataset.py \
      --num-images 1000 \
      --headless
"""

import argparse
import json
import os
import time
from omni.isaac.kit import SimulationApp

parser = argparse.ArgumentParser(description="Grasping Dataset Generator")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
parser.add_argument("--num-images", type=int, default=1000, help="Number of images to generate")
parser.add_argument("--output-dir", type=str, default="~/datasets/grasping_dataset",
                    help="Output directory for dataset")
args, unknown = parser.parse_known_args()

# Launch Isaac Sim
CONFIG = {"headless": args.headless, "width": 1280, "height": 720}
simulation_app = SimulationApp(CONFIG)

import omni
import omni.replicator.core as rep
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.urdf import _urdf
import omni.isaac.core.utils.prims as prim_utils
from pxr import Gf, UsdGeom
import carb

# Enable required extensions
enable_extension("omni.replicator.core")
enable_extension("omni.replicator.isaac")
enable_extension("omni.isaac.urdf")

# Expand output path
output_dir = os.path.expanduser(args.output_dir)


def load_humanoid_robot():
    """
    Load humanoid robot from Chapter 4 URDF.

    Returns:
        Robot prim path or None if failed
    """
    urdf_path = "~/ros2_ws/src/robot-book-code/chapter-04-urdf-sdf/urdf/humanoid_12dof.urdf"
    urdf_path = os.path.expanduser(urdf_path)

    if not os.path.exists(urdf_path):
        carb.log_error(f"URDF not found: {urdf_path}")
        carb.log_error("Please ensure Chapter 4 code is available.")
        return None

    urdf_interface = _urdf.acquire_urdf_interface()
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.default_drive_strength = 5000.0
    import_config.default_position_drive_damping = 500.0

    success, robot_path = urdf_interface.parse_urdf(
        urdf_path, "/World/Humanoid", import_config
    )

    if not success:
        carb.log_error("Failed to import URDF")
        return None

    # Position robot
    robot_prim = prim_utils.get_prim_at_path(robot_path)
    UsdGeom.Xformable(robot_prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))

    carb.log_info(f"Humanoid robot loaded: {robot_path}")
    return robot_path


def setup_scene():
    """
    Setup grasping scene with ground, table, and graspable objects.

    Returns:
        List of graspable object prims
    """
    # Ground plane
    ground = rep.create.plane(
        scale=20.0,
        position=(0, 0, 0),
        semantics=[("class", "ground")]
    )

    # Table (where objects will be placed)
    table = rep.create.cube(
        position=(0.5, 0, 0.3),
        scale=(0.8, 0.6, 0.6),
        semantics=[("class", "table")]
    )

    # Dome light
    light = rep.create.light(
        light_type="Dome",
        intensity=1000.0,
        position=(0, 0, 0)
    )

    # Graspable objects
    cube = rep.create.cube(
        position=(0.5, 0, 0.7),
        scale=0.08,
        semantics=[("class", "cube")]
    )

    cylinder = rep.create.cylinder(
        position=(0.5, 0.15, 0.7),
        scale=(0.05, 0.05, 0.1),
        semantics=[("class", "cylinder")]
    )

    sphere = rep.create.sphere(
        position=(0.5, -0.15, 0.7),
        scale=0.06,
        semantics=[("class", "sphere")]
    )

    carb.log_info("Grasping scene setup complete")
    return [cube, cylinder, sphere]


def register_randomizers(objects: list, robot_path: str = None):
    """
    Register randomization functions for objects, lighting, and robot pose.

    Args:
        objects: List of graspable object prims
        robot_path: Path to robot prim (optional)
    """

    def randomize_objects():
        """Randomize object positions on table and colors."""
        # Randomize positions (table surface is at z=0.6)
        with objects[0]:  # Cube
            rep.modify.pose(
                position=rep.distribution.uniform((0.3, -0.2, 0.65), (0.7, 0.2, 0.75)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )
            rep.randomizer.color(
                colors=rep.distribution.uniform((0.2, 0.2, 0.2), (1.0, 1.0, 1.0))
            )

        with objects[1]:  # Cylinder
            rep.modify.pose(
                position=rep.distribution.uniform((0.3, -0.2, 0.65), (0.7, 0.2, 0.75)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )
            rep.randomizer.color(
                colors=rep.distribution.uniform((0.2, 0.2, 0.2), (1.0, 1.0, 1.0))
            )

        with objects[2]:  # Sphere
            rep.modify.pose(
                position=rep.distribution.uniform((0.3, -0.2, 0.65), (0.7, 0.2, 0.75)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )
            rep.randomizer.color(
                colors=rep.distribution.uniform((0.2, 0.2, 0.2), (1.0, 1.0, 1.0))
            )

        return objects[0].node, objects[1].node, objects[2].node

    def randomize_lighting():
        """Randomize dome light intensity and color temperature."""
        lights = rep.get.prims(path_pattern="/Replicator/Ref_Xform/Ref/DomeLight*")
        with lights:
            rep.modify.attribute(
                "inputs:intensity",
                rep.distribution.uniform(500, 2000)
            )
            # Color temperature simulation (warm to cool)
            rep.modify.attribute(
                "inputs:color",
                rep.distribution.uniform((0.8, 0.8, 1.0), (1.0, 1.0, 1.0))
            )
        return lights.node

    def randomize_table_color():
        """Randomize table color to simulate different surfaces."""
        tables = rep.get.prims(semantics=[("class", "table")])
        with tables:
            rep.randomizer.color(
                colors=rep.distribution.choice([
                    (0.8, 0.7, 0.6),  # Wood
                    (0.9, 0.9, 0.9),  # White
                    (0.3, 0.3, 0.3),  # Dark
                ])
            )
        return tables.node

    # Register randomizers
    rep.randomizer.register(randomize_objects)
    rep.randomizer.register(randomize_lighting)
    rep.randomizer.register(randomize_table_color)

    carb.log_info("Randomizers registered")


def setup_camera():
    """
    Create camera positioned for grasping (robot's perspective).

    Returns:
        render_product: Render product for attaching writers
    """
    # Camera positioned as if mounted on robot's head
    camera = rep.create.camera(
        position=(0.0, 0.5, 1.0),
        look_at=(0.5, 0, 0.7)
    )

    # Render product (640×480 resolution)
    render_product = rep.create.render_product(camera, (640, 480))

    carb.log_info("Camera created at (0, 0.5, 1.0) looking at table")
    return render_product


def setup_writer(render_product, output_dir: str):
    """
    Setup BasicWriter to save RGB, depth, segmentation, and bounding boxes.

    Args:
        render_product: Render product to attach writer to
        output_dir: Directory to save data
    """
    writer = rep.WriterRegistry.get("BasicWriter")

    # Configure output (enable depth for grasping)
    writer.initialize(
        output_dir=output_dir,
        rgb=True,
        depth=True,
        semantic_segmentation=True,
        bounding_box_2d_tight=True,
    )

    # Attach to render product
    writer.attach([render_product])

    carb.log_info(f"Writer attached. Output directory: {output_dir}")
    return writer


def save_metadata(output_dir: str, num_images: int, object_classes: list, elapsed: float):
    """
    Save dataset metadata to JSON.

    Args:
        output_dir: Output directory
        num_images: Number of images generated
        object_classes: List of object class names
        elapsed: Generation time in seconds
    """
    metadata = {
        "dataset": "humanoid_grasping",
        "description": "Synthetic grasping dataset for humanoid manipulation",
        "num_images": num_images,
        "resolution": [640, 480],
        "object_classes": object_classes,
        "generation_time_seconds": elapsed,
        "average_fps": num_images / elapsed,
    }

    metadata_path = os.path.join(output_dir, "metadata.json")
    os.makedirs(os.path.dirname(metadata_path), exist_ok=True)

    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=2)

    carb.log_info(f"Metadata saved to: {metadata_path}")


def run_pipeline(num_images: int):
    """
    Run the Replicator pipeline to generate grasping dataset.

    Args:
        num_images: Number of images to generate
    """
    carb.log_info("=" * 70)
    carb.log_info("GRASPING DATASET GENERATION")
    carb.log_info("=" * 70)
    carb.log_info(f"Images to generate: {num_images}")
    carb.log_info(f"Output directory: {output_dir}")
    carb.log_info("=" * 70)

    # Warmup
    carb.log_info("Warming up (10 frames)...")
    for _ in range(10):
        rep.orchestrator.step()

    # Generate data
    start_time = time.time()

    # Trigger randomization on every frame
    with rep.trigger.on_frame(num_frames=num_images):
        rep.randomizer.randomize_objects()
        rep.randomizer.randomize_lighting()
        rep.randomizer.randomize_table_color()

    # Run orchestrator
    rep.orchestrator.run()

    # Wait for completion
    while not rep.orchestrator.get_is_stopped():
        simulation_app.update()

    elapsed = time.time() - start_time

    # Save metadata
    object_classes = ["cube", "cylinder", "sphere"]
    save_metadata(output_dir, num_images, object_classes, elapsed)

    carb.log_info("")
    carb.log_info("=" * 70)
    carb.log_info("DATASET GENERATION COMPLETE")
    carb.log_info("=" * 70)
    carb.log_info(f"Total Images:           {num_images}")
    carb.log_info(f"Output Directory:       {output_dir}")
    carb.log_info(f"Generation Time:        {elapsed:.1f} seconds")
    carb.log_info(f"Average FPS:            {num_images / elapsed:.1f}")
    carb.log_info("=" * 70)
    carb.log_info("Object Classes:")
    for i, obj_class in enumerate(object_classes, 1):
        carb.log_info(f"  - {obj_class} (ID: {i}): {num_images} instances")
    carb.log_info("=" * 70)
    carb.log_info("")
    carb.log_info("Output structure:")
    carb.log_info(f"  {output_dir}/rgb/")
    carb.log_info(f"  {output_dir}/depth/")
    carb.log_info(f"  {output_dir}/semantic_segmentation/")
    carb.log_info(f"  {output_dir}/bounding_box_2d_tight/")
    carb.log_info(f"  {output_dir}/bounding_box_2d_tight.npy")
    carb.log_info(f"  {output_dir}/metadata.json")
    carb.log_info("=" * 70)


def main():
    """Main function."""

    # Load humanoid robot (optional - can be disabled for faster generation)
    # robot_path = load_humanoid_robot()

    # Setup scene
    objects = setup_scene()

    # Register randomizers
    register_randomizers(objects)

    # Setup camera and writer
    render_product = setup_camera()
    writer = setup_writer(render_product, output_dir)

    # Run pipeline
    run_pipeline(args.num_images)

    # Cleanup
    simulation_app.close()


if __name__ == "__main__":
    main()

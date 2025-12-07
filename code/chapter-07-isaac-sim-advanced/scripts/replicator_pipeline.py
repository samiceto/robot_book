#!/usr/bin/env python3
"""
Complete Replicator Pipeline for Synthetic Data Generation.

This script demonstrates Isaac Sim's Replicator API for procedural scene
generation and synthetic data capture with automatic annotations.

Outputs:
- RGB images (640×480 PNG)
- Semantic segmentation masks (PNG with class IDs)
- 2D bounding boxes (NumPy arrays)
- Depth maps (16-bit PNG or float32 .npy)

Usage:
    cd ~/.local/share/ov/pkg/isaac-sim-4.2.0

    # Generate 100 frames
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/replicator_pipeline.py \
      --num-frames 100

    # Custom output directory
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/replicator_pipeline.py \
      --output-dir ~/datasets/my_dataset

    # Headless mode (faster)
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/replicator_pipeline.py \
      --headless --num-frames 1000
"""

import argparse
import os
import time
from omni.isaac.kit import SimulationApp

parser = argparse.ArgumentParser(description="Replicator Pipeline Demo")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
parser.add_argument("--num-frames", type=int, default=100, help="Number of frames to generate")
parser.add_argument("--output-dir", type=str, default="~/datasets/replicator_output",
                    help="Output directory for dataset")
args, unknown = parser.parse_known_args()

# Launch Isaac Sim
CONFIG = {"headless": args.headless, "width": 1280, "height": 720}
simulation_app = SimulationApp(CONFIG)

import omni
import omni.replicator.core as rep
from omni.isaac.core.utils.extensions import enable_extension
import carb

# Enable required extensions
enable_extension("omni.replicator.core")
enable_extension("omni.replicator.isaac")

# Expand output path
output_dir = os.path.expanduser(args.output_dir)


def setup_scene():
    """
    Setup a simple scene with ground, lighting, and randomizable objects.
    """
    # Ground plane
    ground = rep.create.plane(
        scale=10.0,
        position=(0, 0, 0),
        semantics=[("class", "ground")]
    )

    # Dome light (randomized later)
    light = rep.create.light(
        light_type="Dome",
        intensity=1000.0,
        position=(0, 0, 0),
        rotation=(0, 0, 0)
    )

    # Create randomizable objects
    cube = rep.create.cube(
        position=(0, 0, 0.5),
        scale=0.5,
        semantics=[("class", "cube")]
    )

    cylinder = rep.create.cylinder(
        position=(1.5, 0, 0.5),
        scale=(0.5, 0.5, 0.5),
        semantics=[("class", "cylinder")]
    )

    sphere = rep.create.sphere(
        position=(-1.5, 0, 0.5),
        scale=0.5,
        semantics=[("class", "sphere")]
    )

    carb.log_info("Scene setup complete")
    return ground, light, [cube, cylinder, sphere]


def register_randomizers(objects: list):
    """
    Register randomization functions for objects and lighting.

    Args:
        objects: List of object prims to randomize
    """

    def randomize_objects():
        """Randomize object positions, rotations, and colors."""
        with objects[0]:  # Cube
            rep.modify.pose(
                position=rep.distribution.uniform((-1.0, -1.0, 0.5), (1.0, 1.0, 1.5)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )
            rep.randomizer.color(
                colors=rep.distribution.uniform((0.2, 0.2, 0.2), (1.0, 1.0, 1.0))
            )

        with objects[1]:  # Cylinder
            rep.modify.pose(
                position=rep.distribution.uniform((-1.0, -1.0, 0.5), (1.0, 1.0, 1.5)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )
            rep.randomizer.color(
                colors=rep.distribution.uniform((0.2, 0.2, 0.2), (1.0, 1.0, 1.0))
            )

        with objects[2]:  # Sphere
            rep.modify.pose(
                position=rep.distribution.uniform((-1.0, -1.0, 0.5), (1.0, 1.0, 1.5)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )
            rep.randomizer.color(
                colors=rep.distribution.uniform((0.2, 0.2, 0.2), (1.0, 1.0, 1.0))
            )

        return objects[0].node, objects[1].node, objects[2].node

    def randomize_lighting():
        """Randomize dome light intensity."""
        lights = rep.get.prims(path_pattern="/Replicator/Ref_Xform/Ref/DomeLight*")
        with lights:
            rep.modify.attribute(
                "inputs:intensity",
                rep.distribution.uniform(500, 2000)
            )
        return lights.node

    # Register randomizers
    rep.randomizer.register(randomize_objects)
    rep.randomizer.register(randomize_lighting)

    carb.log_info("Randomizers registered")


def setup_camera():
    """
    Create camera and render product for data capture.

    Returns:
        render_product: Render product for attaching writers
    """
    # Camera positioned to view the scene
    camera = rep.create.camera(
        position=(5.0, 5.0, 3.0),
        look_at=(0, 0, 0.5)
    )

    # Render product (640×480 resolution)
    render_product = rep.create.render_product(camera, (640, 480))

    carb.log_info("Camera created at (5, 5, 3) looking at (0, 0, 0.5)")
    return render_product


def setup_writer(render_product, output_dir: str):
    """
    Setup BasicWriter to save RGB, segmentation, and bounding boxes.

    Args:
        render_product: Render product to attach writer to
        output_dir: Directory to save data
    """
    writer = rep.WriterRegistry.get("BasicWriter")

    # Configure output
    writer.initialize(
        output_dir=output_dir,
        rgb=True,
        semantic_segmentation=True,
        bounding_box_2d_tight=True,
        distance_to_camera=False,  # Optional: enable for depth maps
    )

    # Attach to render product
    writer.attach([render_product])

    carb.log_info(f"Writer attached. Output directory: {output_dir}")
    return writer


def run_pipeline(num_frames: int):
    """
    Run the Replicator pipeline to generate synthetic data.

    Args:
        num_frames: Number of frames to generate
    """
    carb.log_info("=" * 70)
    carb.log_info("STARTING DATA GENERATION")
    carb.log_info("=" * 70)
    carb.log_info(f"Frames to generate: {num_frames}")
    carb.log_info(f"Output directory: {output_dir}")
    carb.log_info("=" * 70)

    # Warmup (ensure rendering is initialized)
    carb.log_info("Warming up (10 frames)...")
    for _ in range(10):
        rep.orchestrator.step()

    # Generate data
    start_time = time.time()

    # Trigger randomization on every frame
    with rep.trigger.on_frame(num_frames=num_frames):
        rep.randomizer.randomize_objects()
        rep.randomizer.randomize_lighting()

    # Run orchestrator
    rep.orchestrator.run()

    # Wait for completion
    while not rep.orchestrator.get_is_stopped():
        simulation_app.update()

    elapsed = time.time() - start_time

    carb.log_info("")
    carb.log_info("=" * 70)
    carb.log_info("GENERATION COMPLETE")
    carb.log_info("=" * 70)
    carb.log_info(f"Total frames: {num_frames}")
    carb.log_info(f"Time elapsed: {elapsed:.2f} seconds")
    carb.log_info(f"Average FPS: {num_frames / elapsed:.2f}")
    carb.log_info(f"Output directory: {output_dir}")
    carb.log_info("=" * 70)
    carb.log_info("")
    carb.log_info("Output structure:")
    carb.log_info(f"  {output_dir}/rgb/")
    carb.log_info(f"  {output_dir}/semantic_segmentation/")
    carb.log_info(f"  {output_dir}/bounding_box_2d_tight/")
    carb.log_info(f"  {output_dir}/bounding_box_2d_tight.npy  (aggregated)")
    carb.log_info("=" * 70)


def main():
    """Main function."""

    # Setup scene
    ground, light, objects = setup_scene()

    # Register randomizers
    register_randomizers(objects)

    # Setup camera and writer
    render_product = setup_camera()
    writer = setup_writer(render_product, output_dir)

    # Run pipeline
    run_pipeline(args.num_frames)

    # Cleanup
    simulation_app.close()


if __name__ == "__main__":
    main()

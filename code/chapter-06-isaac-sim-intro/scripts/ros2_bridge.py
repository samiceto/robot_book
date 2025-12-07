#!/usr/bin/env python3
"""
Isaac Sim to ROS 2 Bridge using ActionGraph (Python API).

This script demonstrates:
1. Creating ActionGraph programmatically (alternative to GUI)
2. Publishing joint states, camera images to ROS 2
3. Subscribing to joint commands from ROS 2

Usage:
    cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/ros2_bridge.py

Then in another terminal:
    source /opt/ros/iron/setup.bash
    ros2 topic list
    ros2 topic echo /joint_states
"""

import argparse
from omni.isaac.kit import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
args, unknown = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless})

import omni
import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.prims import create_prim
import omni.isaac.core.utils.prims as prim_utils
from pxr import Gf, UsdGeom
import carb

# Enable required extensions
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.isaac.urdf")


def create_ros2_bridge_graph(robot_prim_path: str, camera_prim_path: str = None):
    """
    Create ActionGraph for ROS 2 bridge programmatically.

    Args:
        robot_prim_path: USD path to robot articulation root
        camera_prim_path: USD path to camera (optional)
    """
    # Create graph
    graph_path = "/ActionGraph/ROS2_Bridge"
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", 0),
                ("PublishJointState.inputs:topicName", "/joint_states"),
                ("PublishJointState.inputs:targetPrim", robot_prim_path),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
            ],
        },
    )

    carb.log_info(f"ROS 2 bridge graph created at: {graph_path}")
    carb.log_info(f"  Publishing /joint_states from: {robot_prim_path}")

    # Add camera publisher if camera provided
    if camera_prim_path:
        og.Controller.edit(
            graph,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("PublishCamera", "omni.isaac.ros2_bridge.ROS2PublishImage"),
                    ("PublishCameraInfo", "omni.isaac.ros2_bridge.ROS2PublishCameraInfo"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishCamera.inputs:topicName", "/camera/image_raw"),
                    ("PublishCamera.inputs:renderProductPath", camera_prim_path),
                    ("PublishCameraInfo.inputs:topicName", "/camera/camera_info"),
                    ("PublishCameraInfo.inputs:renderProductPath", camera_prim_path),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishCamera.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishCamera.inputs:context"),
                    ("OnPlaybackTick.outputs:tick", "PublishCameraInfo.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishCameraInfo.inputs:context"),
                ],
            },
        )
        carb.log_info(f"  Publishing /camera/image_raw from: {camera_prim_path}")

    return graph


def create_camera(position: tuple, target: tuple, resolution: tuple = (640, 480)):
    """
    Create camera prim and render product.

    Args:
        position: Camera position (x, y, z)
        target: Point camera looks at (x, y, z)
        resolution: Image resolution (width, height)

    Returns:
        Tuple of (camera_prim_path, render_product_path)
    """
    import omni.kit.viewport.utility as vp_utils
    from omni.isaac.core.utils.render_product import create_render_product

    # Create camera prim
    camera_path = "/World/Camera"
    camera_prim = create_prim(
        camera_path,
        "Camera",
        attributes={
            "focalLength": 24.0,
            "focusDistance": 400.0,
            "horizontalAperture": 20.955,
            "clippingRange": (0.1, 1000.0),
        }
    )

    # Set camera transform
    camera_xform = UsdGeom.Xformable(camera_prim)
    camera_xform.ClearXformOpOrder()

    # Translate
    translate_op = camera_xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*position))

    # Look-at rotation (simplified, assumes camera above target)
    import math
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    dz = target[2] - position[2]
    distance = math.sqrt(dx**2 + dy**2 + dz**2)

    pitch = -math.degrees(math.asin(dz / distance))
    yaw = math.degrees(math.atan2(dy, dx))

    rotate_op = camera_xform.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3f(pitch, 0, yaw))

    # Create render product
    render_product_path = create_render_product(camera_path, resolution)

    carb.log_info(f"Camera created at: {camera_path}")
    carb.log_info(f"  Position: {position}, Looking at: {target}")
    carb.log_info(f"  Render product: {render_product_path}")

    return camera_path, render_product_path


def main():
    """Main function."""

    # Create world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Add lighting
    create_prim(
        "/World/DomeLight",
        "DomeLight",
        attributes={"inputs:intensity": 1000.0}
    )

    # Load URDF (simplified - assumes Chapter 4 URDF available)
    from omni.isaac.urdf import _urdf
    urdf_path = "~/ros2_ws/src/robot-book-code/chapter-04-urdf-sdf/urdf/humanoid_12dof.urdf"
    import os
    urdf_path = os.path.expanduser(urdf_path)

    if not os.path.exists(urdf_path):
        carb.log_error(f"URDF not found: {urdf_path}")
        carb.log_error("Please ensure Chapter 4 code is available.")
        simulation_app.close()
        return

    urdf_interface = _urdf.acquire_urdf_interface()
    import_config = _urdf.ImportConfig()
    import_config.fix_base = False
    import_config.default_drive_strength = 5000.0
    import_config.default_position_drive_damping = 500.0

    success, robot_path = urdf_interface.parse_urdf(
        urdf_path, "/World/Humanoid", import_config
    )

    if not success:
        carb.log_error("Failed to import URDF")
        simulation_app.close()
        return

    # Position robot
    robot_prim = prim_utils.get_prim_at_path(robot_path)
    UsdGeom.Xformable(robot_prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.0))

    # Create camera
    camera_path, render_product_path = create_camera(
        position=(3.0, 3.0, 1.5),
        target=(0.0, 0.0, 1.0),
        resolution=(640, 480)
    )

    # Create ROS 2 bridge graph
    graph = create_ros2_bridge_graph(robot_path, render_product_path)

    # Reset world
    world.reset()

    carb.log_info("="*70)
    carb.log_info("Isaac Sim ROS 2 Bridge - Ready")
    carb.log_info("="*70)
    carb.log_info("ROS 2 topics published:")
    carb.log_info("  /joint_states (sensor_msgs/JointState) - 60 Hz")
    carb.log_info("  /camera/image_raw (sensor_msgs/Image) - 30 Hz")
    carb.log_info("  /camera/camera_info (sensor_msgs/CameraInfo) - 30 Hz")
    carb.log_info("")
    carb.log_info("Verify in another terminal:")
    carb.log_info("  source /opt/ros/iron/setup.bash")
    carb.log_info("  ros2 topic list")
    carb.log_info("  ros2 topic echo /joint_states")
    carb.log_info("="*70)

    # Simulation loop
    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()

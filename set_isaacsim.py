# fmt: off
import os
import sys
import argparse
import yaml
import subprocess, textwrap, json
import numpy as np
# Make your workspace importable (arena_simulation_setup, isaac_utils, etc.)
WORKSPACE_DIR = "/home/ewhaglab/isaac"
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)

from isaacsim import SimulationApp

# NOTE: headless must be set BEFORE SimulationApp is created.
CONFIG = {"renderer": "Wireframe", "headless": False}
simulation_app = SimulationApp(CONFIG)
simulation_app.update()

from pxr import UsdPhysics, PhysxSchema, Gf, Sdf, UsdGeom
import carb
import omni.usd
from omni.isaac.core.utils import extensions
extensions.enable_extension("isaacsim.ros2.bridge")
from isaac_utils.robots.jackal.jackal import Jackal
import arena_simulation_setup.entities.robot
from isaac_utils.sensors import (
    lidar_setup,
    publish_lidar,
    imu_setup, 
    publish_imu,
    camera_setup,
    publish_rgb,
    publish_depth,
    publish_camera_info,
    publish_camera_tf,
)
# SimulationContext import (version-safe)
try:
    from isaacsim.core.api import SimulationContext
except Exception:
    from omni.isaac.core import SimulationContext  # fallback

# graphs import (prefer control/..., fallback to top-level)
def _import_graph(name: str):
    try:
        mod = __import__(f"isaac_utils.graphs.control.{name}", fromlist=[name])
        return mod
    except Exception:
        mod = __import__(f"isaac_utils.graphs.{name}", fromlist=[name])
        return mod

# Load graph functions
time_mod = _import_graph("time")
odom_mod = _import_graph("odom")
tf_mod = _import_graph("tf")
js_mod = _import_graph("joint_states")
diff_mod = _import_graph("differential")

PublishTime = getattr(time_mod, "PublishTime")
odom = getattr(odom_mod, "odom")
tf = getattr(tf_mod, "tf")
joint_states = getattr(js_mod, "joint_states")
differential = getattr(diff_mod, "differential")

# door_manager optional
try:
    from isaac_utils.managers.door_manager import door_manager
except Exception:
    door_manager = None
# fmt: on


# -------------------------
# YAML utils
# -------------------------
def load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def abs_path(p: str) -> str:
    if not p:
        return p
    return p if os.path.isabs(p) else os.path.abspath(p)

# -------------------------
# Extensions (ROS2 bridge name changed across versions)
# -------------------------
def enable_ros2_bridge():
    # Newer Isaac Sim
    try:
        extensions.enable_extension("isaacsim.ros2.bridge")
        carb.log_info("Enabled extension: isaacsim.ros2.bridge")
        return
    except Exception as e:
        carb.log_warn(f"Failed enabling isaacsim.ros2.bridge: {e}")

def enable_required_extensions():
    # Core graph nodes
    extensions.enable_extension("omni.graph.nodes")
    extensions.enable_extension("isaacsim.core.nodes")

    # ROS2 bridge (version safe)
    enable_ros2_bridge()

    # Differential controller node
    extensions.enable_extension("omni.isaac.wheeled_robots")

    extensions.enable_extension("isaacsim.sensors.physics")
    # (optional) navmesh extension if you later need it
    try:
        extensions.enable_extension("omni.anim.navigation.core")
    except Exception:
        pass

    simulation_app.update()


# -------------------------
# USD helpers
# -------------------------
def ensure_xform(stage, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    if prim and prim.IsValid():
        return prim
    return stage.DefinePrim(prim_path, "Xform")


def add_usd_reference(stage, prim_path: str, usd_path: str):
    prim = ensure_xform(stage, prim_path)
    refs = prim.GetReferences()
    refs.ClearReferences()
    refs.AddReference(usd_path)
    return prim


def set_xform_pose(stage, prim_path: str, position_xyz, quat_wxyz):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found for pose set: {prim_path}")

    xformable = UsdGeom.Xformable(prim)

    x, y, z = [float(v) for v in position_xyz]
    w, qx, qy, qz = [float(v) for v in quat_wxyz]

    t_op = None
    o_op = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and t_op is None:
            t_op = op
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient and o_op is None:
            o_op = op

    if t_op is None:
        t_op = xformable.AddTranslateOp()  # default float3
    t_op.Set(Gf.Vec3f(x, y, z))

    if o_op is None:
        o_op = xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat)

    type_name = o_op.GetAttr().GetTypeName()
    if str(type_name) == "quatd":
        o_op.Set(Gf.Quatd(w, Gf.Vec3d(qx, qy, qz)))
    else:
        o_op.Set(Gf.Quatf(w, Gf.Vec3f(qx, qy, qz)))

# -------------------------
# Build world/robot
# -------------------------
def build_world(world_cfg: dict):
    stage = omni.usd.get_context().get_stage()

    # Optionally reference a world USD
    world_usd = abs_path(world_cfg.get("world", {}).get("usd_path", ""))
    world_prim_path = world_cfg.get("world", {}).get("prim_path", "/World/Environment")
    if world_usd:
        if not os.path.exists(world_usd):
            raise FileNotFoundError(f"world.usd not found: {world_usd}")
        add_usd_reference(stage, world_prim_path, world_usd)
    simulation_app.update()

def build_robot_and_ros(robot_cfg: dict):
    stage = omni.usd.get_context().get_stage()

    name = robot_cfg.get("name", "jackal")
    robot_block = robot_cfg.get("robot", {})
    robot_usd = abs_path(robot_block.get("usd_path", "")) 
    robot_prim_path = robot_block.get("prim_path", f"/World/Robots/{name}")
    ros_cfg = robot_cfg.get("ros", {})
    publish_clock = bool(ros_cfg.get("publish_clock", True))

    if publish_clock:
        PublishTime("/World/publish_time")

    if not robot_usd or not os.path.exists(robot_usd):
        raise FileNotFoundError(f"robot.usd not found: {robot_usd}")

    add_usd_reference(stage, robot_prim_path, robot_usd)

    # Pose
    pose = robot_block.get("pose", {})
    pos = pose.get("position", [3.0, 0.0, 0.0])
    quat = pose.get("orientation_quat_wxyz", [1.0, 0.0, 0.0, 0.0]) 
    set_xform_pose(stage, robot_prim_path, pos, quat)

    simulation_app.update()

    model_name = robot_cfg.get("model", name)
    robot_params = arena_simulation_setup.entities.robot.Robot(model_name).model_params  # :contentReference[oaicite:1]{index=1}

    #Jackal OmniGraph
    j = Jackal(name=name, prim_path=robot_prim_path)

    j.control_and_publish_joint_states()
    j.publish_odom_and_tf()

    # Sensors
    sensors_cfg = robot_cfg.get("sensors", {}) or {}
    ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", ros_cfg.get("domain_id", 30)))

    # Link prim to attach sensors (best-effort default)
    frames_cfg = robot_cfg.get("frames", {}) or {}
    base_frame_id = frames_cfg.get("base_frame_id", "base_link")
    base_link_prim = sensors_cfg.get("base_link_prim", f"{robot_prim_path}/{base_frame_id}")


    # --- LiDAR ---
    lidar_cfg = sensors_cfg.get("lidar", {}) or {}
    if lidar_cfg.get("enabled", True):
        lidar_prim_path = lidar_cfg.get("prim_path")
        #lidar, prim = lidar_setup(lidar_cfg.get("parent_prim_path"), "Lidar")
        publish_lidar(model_name, lidar_prim_path)

    # --- IMU --- 
    imu_cfg = sensors_cfg.get("imu", {}) or {}
    if imu_cfg.get("enabled", True):
        imu_prim_path = imu_cfg.get("prim_path")
        imu = imu_setup(imu_cfg.get("parent_prim_path"), "imu_sensor")
        publish_imu(model_name, imu_prim_path, imu)

#     # --- Camera ---
#     cam_cfg = sensors_cfg.get("camera", {}) or {}
#     if cam_cfg.get("enabled", True):
#         cam_name = cam_cfg.get("name", "camera")
#         cam_freq = float(cam_cfg.get("freq", 10))

#         cam = camera_setup(robot_prim_path, cam_name)

#         # Publish TF + topics
#         if bool(cam_cfg.get("publish_tf", True)):
#             publish_camera_tf(cam)

#         if bool(cam_cfg.get("publish_rgb", True)):
#             publish_rgb(cam, cam_freq)

#         if bool(cam_cfg.get("publish_depth", False)):
#             publish_depth(cam, cam_freq)

#         if bool(cam_cfg.get("publish_camera_info", True)):
#             publish_camera_info(cam, cam_freq)

#     simulation_app.update()

# -------------------------
# Main
# -------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True, help="path to world.yaml")
    parser.add_argument("--robot", required=True, help="path to robot.yaml")
    args = parser.parse_args()

    enable_required_extensions()

    # Create a fresh stage (optional but recommended for repeatability)
    omni.usd.get_context().new_stage()
    simulation_app.update()

    world_cfg = load_yaml(args.world)
    robot_cfg = load_yaml(args.robot)

    # Simulation context (no World used)
    stage_units = float(world_cfg.get("stage_units_in_meters", 1.0))
    sim_context = SimulationContext(stage_units_in_meters=stage_units)

    # Build
    build_world(world_cfg)
    build_robot_and_ros(robot_cfg)

    # Reset sim (safe)
    try:
        sim_context.reset()
    except Exception:
        pass

    # Main loop
    while simulation_app.is_running():
        simulation_app.update()
        if door_manager is not None:
            try:
                door_manager.update()
            except Exception:
                pass

    simulation_app.close()


if __name__ == "__main__":
    main()
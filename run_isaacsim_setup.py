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
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)
simulation_app.update()
import omni.kit.app
from omni.isaac.core.world import World  

# NOTE: switched from legacy `pedestrian` module to `people` extension APIs
from people.settings import PeopleSettings
from isaac_utils.utils.assets import get_assets_root_path_safe

# default world settings (previously from pedestrian.simulator.params)
DEFAULT_WORLD_SETTINGS = {
    "physics_dt": 1.0 / 250.0,
    "stage_units_in_meters": 1.0,
    "rendering_dt": 1.0 / 60.0,
    "device": "gpu",
}
SIMULATION_ENVIRONMENTS = {}

from pxr import UsdPhysics, PhysxSchema, Gf, Sdf, UsdGeom, Usd, UsdSkel
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
#from isaac_utils.pub_pedestrian import publish_pedestrians
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
    extensions.enable_extension("omni.graph.scriptnode") 
    extensions.enable_extension("isaacsim.core.nodes")

    # ROS2 bridge (version safe)
    enable_ros2_bridge()

    # Differential controller node
    extensions.enable_extension("omni.isaac.wheeled_robots")

    extensions.enable_extension("isaacsim.sensors.physics")
    extensions.enable_extension("omni.anim.graph.core")
    extensions.enable_extension("omni.anim.people")
    simulation_app.update()
    simulation_app.update()
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


# -------------------------
# Pedestrian support
# -------------------------
# def _get_character_usd_path(character_name: str) -> str | None:
#     """Resolve a character-name -> USD asset path using PeopleSettings.CHARACTER_ASSETS_PATH or Isaac assets."""
#     settings = carb.settings.get_settings()
#     assets_root = settings.get(PeopleSettings.CHARACTER_ASSETS_PATH)
#     if not assets_root:
#         assets_root = get_assets_root_path_safe()
#         assets_root = os.path.join(assets_root, "Isaac/People/Characters")

#     # candidate folder on the asset server/local filesystem
#     folder = os.path.join(assets_root, character_name)
#     try:
#         result, props = omni.client.stat(folder)
#         if result != omni.client.Result.OK:
#             return None
#     except Exception:
#         return None

#     # find a .usd inside that folder
#     try:
#         result, listing = omni.client.list(folder)
#         if result != omni.client.Result.OK:
#             return None
#         for item in listing:
#             if item.relative_path.endswith(".usd"):
#                 return f"{folder}/{item.relative_path}"
#     except Exception:
#         return None

#     return None


# def spawn_pedestrians(world, pedestrian_cfg: dict) -> list:
#     """Spawn pedestrians using the `people` extension (CharacterBehavior + NavigationManager).

#     - uses PeopleSettings.CHARACTER_PRIM_PATH as the parent prim
#     - attaches CharacterBehavior to spawned SkelRoot so navigation/avoidance/commands work
#     """

#     people_spawned = []
#     pedestrians = pedestrian_cfg.get("pedestrians", [])

#     # spawn each character USD under the CHARACTER_PRIM_PATH
#     characters_root = carb.settings.get_settings().get(PeopleSettings.CHARACTER_PRIM_PATH) or "/World/Characters"
#     stage = omni.usd.get_context().get_stage()

#     for ped_cfg in pedestrians:
#         try:
#             name = ped_cfg.get("name", "pedestrian_0")
#             character = ped_cfg.get("character")
#             pos = ped_cfg.get("position", [0.0, 0.0, 0.1])
#             yaw = float(ped_cfg.get("yaw", 0.0))

#             usd_path = _get_character_usd_path(character) if character else None
#             if not usd_path:
#                 carb.log_warn(f"Character USD for '{character}' not found; attempting fallback list")
#                 # list available characters from the configured assets root
#                 assets_root = carb.settings.get_settings().get(PeopleSettings.CHARACTER_ASSETS_PATH) or get_assets_root_path_safe()
#                 assets_root = os.path.join(assets_root, "Isaac/People/Characters") if "Isaac/People/Characters" not in assets_root else assets_root
#                 try:
#                     result, listing = omni.client.list(assets_root)
#                     if result == omni.client.Result.OK and listing:
#                         character = listing[0].relative_path
#                         usd_path = _get_character_usd_path(character)
#                 except Exception:
#                     pass

#             if not usd_path:
#                 carb.log_error(f"No USD found for character '{character}' — skipping spawn for {name}")
#                 continue

#             prim_path = f"{characters_root}/{name}"
#             add_usd_reference(stage, prim_path, usd_path)
#             set_xform_pose(stage, prim_path, pos, [1.0, 0.0, 0.0, 0.0])
#             carb.log_info(f"Spawned character prim {prim_path} -> {usd_path}")

#             # attach CharacterBehavior script to SkelRoot
#             skelroot_prim_path = None
#             root_prim = stage.GetPrimAtPath(characters_root)
#             if root_prim and root_prim.IsValid():
#                 for child in root_prim.GetChildren():
#                     if child.GetName() == name:
#                         for p in Usd.PrimRange(child):
#                             if p.GetTypeName() == "SkelRoot":
#                                 skelroot_prim_path = p.GetPath()
#                                 break
#                         if skelroot_prim_path:
#                             break

#             if skelroot_prim_path:
#                 script_path = "/ewhaglab/isaac/people/scripts/character_behavior.py"
#                 omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[Sdf.Path(skelroot_prim_path)])
#                 attr = stage.GetPrimAtPath(skelroot_prim_path).GetAttribute("omni:scripting:scripts")
#                 script_list = attr.Get() or []
#                 if script_path not in script_list:
#                     script_list.append(script_path)
#                     attr.Set(script_list)
#                 carb.log_info(f"Attached CharacterBehavior to {skelroot_prim_path}")
#                 people_spawned.append({"name": name, "prim_path": prim_path, "skelroot_path": str(skelroot_prim_path)})
#             else:
#                 carb.log_warn(f"SkelRoot for {name} not found — CharacterBehavior not attached")
#                 people_spawned.append({"name": name, "prim_path": prim_path})

#         except Exception as e:
#             carb.log_error(f"Failed to spawn pedestrian {ped_cfg.get('name', 'unknown')}: {e}")
#             continue

#     # debug: list managed characters known to omni.anim.people
#     try:
#         from people.scripts.global_character_position_manager import GlobalCharacterPositionManager
#         mgr = GlobalCharacterPositionManager.get_instance()
#         carb.log_info(f"GlobalCharacterPositionManager entries after spawn: {list(mgr.get_all_managed_characters())}")
#     except Exception:
#         pass

#     return people_spawned

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
    # determine base_frame_id early so DynamicObstacle attach can use it
    frames_cfg = robot_block.get("frames", {}) or {}
    base_frame_id = frames_cfg.get("base_frame_id", "base_link")

    # ensure model_name is defined before using robot_params
    model_name = robot_cfg.get("model", name)
    robot_params = arena_simulation_setup.entities.robot.Robot(model_name).model_params  # :contentReference[oaicite:1]{index=1}

    #Jackal OmniGraph
    j = Jackal(name=name, prim_path=robot_prim_path)

    j.control_and_publish_joint_states()
    j.publish_odom_and_tf()

    # # Sensors
    # sensors_cfg = robot_cfg.get("sensors", {}) or {}

    # # LiDAR
    # lidar_cfg = sensors_cfg.get("lidar", {}) or {}
    # if lidar_cfg.get("enabled", True):
    #     lidar_prim_path = lidar_cfg.get("prim_path")
    #     lidar = lidar_setup(lidar_cfg.get("parent_prim_path"), "Lidar")
    #     publish_lidar(model_name, lidar_prim_path, lidar)

    # # IMU
    # imu_cfg = sensors_cfg.get("imu", {}) or {}
    # if imu_cfg.get("enabled", True):
    #     imu_prim_path = imu_cfg.get("prim_path")
    #     imu = imu_setup(imu_cfg.get("parent_prim_path"), "imu_sensor")
    #     publish_imu(model_name, imu_prim_path, imu)

    simulation_app.update()

# -------------------------
# Main
# -------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True, help="path to world.yaml")
    parser.add_argument("--robot", required=True, help="path to robot.yaml")
    parser.add_argument("--pedestrians", default=None, help="path to pedestrians.yaml")
    args = parser.parse_args()

    enable_required_extensions()

    # Create a fresh stage (optional but recommended for repeatability)
    omni.usd.get_context().new_stage()
    simulation_app.update()
    # switched to people extension for character behavior and navigation
    world_cfg = load_yaml(args.world)
    robot_cfg = load_yaml(args.robot)
    pedestrian_cfg = load_yaml(args.pedestrians) if args.pedestrians else {}
    
    # Simulation context
    stage_units = float(world_cfg.get("stage_units_in_meters", 1.0))
    sim_context = SimulationContext(stage_units_in_meters=stage_units)
    
    # Create World with default settings
    world_settings = {**DEFAULT_WORLD_SETTINGS}
    world = World(**world_settings)

    # Build
    build_world(world_cfg)
    build_robot_and_ros(robot_cfg)
    simulation_app.update()

    #sim_context.reset()
    # if pedestrian_cfg.get("pedestrians"):
    #     spawn_pedestrians(world, pedestrian_cfg)
    #     simulation_app.update() 
    #     simulation_app.update()
    #     sim_context.play()

    # Main loop
    while simulation_app.is_running():
        world.step(render=True)
        if door_manager is not None:
            try:
                door_manager.update()
            except Exception:
                pass

    simulation_app.close()


if __name__ == "__main__":
    main()

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
def _get_character_usd_path(character_name: str) -> str | None:
    """Resolve a character-name -> USD asset path using PeopleSettings.CHARACTER_ASSETS_PATH or Isaac assets."""
    settings = carb.settings.get_settings()
    assets_root = settings.get(PeopleSettings.CHARACTER_ASSETS_PATH)
    if not assets_root:
        assets_root = get_assets_root_path_safe()
        assets_root = os.path.join(assets_root, "Isaac/People/Characters")

    # candidate folder on the asset server/local filesystem
    folder = os.path.join(assets_root, character_name)
    try:
        result, props = omni.client.stat(folder)
        if result != omni.client.Result.OK:
            return None
    except Exception:
        return None

    # find a .usd inside that folder
    try:
        result, listing = omni.client.list(folder)
        if result != omni.client.Result.OK:
            return None
        for item in listing:
            if item.relative_path.endswith(".usd"):
                return f"{folder}/{item.relative_path}"
    except Exception:
        return None

    return None


def _configure_people_navmesh(navmesh_cfg: dict):
    """Set navmesh / avoidance settings (replaces old PeopleManager.rebuild_nav_mesh)."""
    height = float(navmesh_cfg.get("agent_height", 1.7))
    radius = float(navmesh_cfg.get("agent_radius", 0.35))
    exclude_rigid_bodies = bool(navmesh_cfg.get("exclude_rigid_bodies", False))
    auto_rebake_on_changes = bool(navmesh_cfg.get("auto_rebake_on_changes", False))
    auto_rebake_delay_seconds = int(navmesh_cfg.get("auto_rebake_delay_seconds", 4))
    view_nav_mesh = bool(navmesh_cfg.get("view_navmesh", False))

    omni.kit.commands.execute('ChangeSetting', path='/exts/omni.anim.navigation.core/navMesh/config/agentHeight', value=height)
    omni.kit.commands.execute('ChangeSetting', path='/exts/omni.anim.navigation.core/navMesh/config/agentRadius', value=radius)
    omni.kit.commands.execute('ChangeSetting', path='/persistent/exts/omni.anim.navigation.core/navMesh/autoRebakeOnChanges', value=auto_rebake_on_changes)
    omni.kit.commands.execute('ChangeSetting', path='/persistent/exts/omni.anim.navigation.core/navMesh/autoRebakeDelaySeconds', value=auto_rebake_delay_seconds)
    omni.kit.commands.execute('ChangeSetting', path='/exts/omni.anim.navigation.core/navMesh/config/excludeRigidBodies', value=exclude_rigid_bodies)
    omni.kit.commands.execute('ChangeSetting', path='/persistent/exts/omni.anim.navigation.core/navMesh/viewNavMesh', value=view_nav_mesh)
    omni.kit.commands.execute('ChangeSetting', path='/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled', value=bool(navmesh_cfg.get('dynamic_avoidance_enabled', True)))
    omni.kit.commands.execute('ChangeSetting', path='/exts/omni.anim.people/navigation_settings/navmesh_enabled', value=bool(navmesh_cfg.get('navmesh_enabled', True)))


def spawn_pedestrians(world, pedestrian_cfg: dict) -> list:
    """Spawn pedestrians using the `people` extension (CharacterBehavior + NavigationManager).

    - uses PeopleSettings.CHARACTER_PRIM_PATH as the parent prim
    - writes an on-stage command file (optional) from YAML `target_position` entries and sets PeopleSettings.COMMAND_FILE_PATH
    - attaches CharacterBehavior to spawned SkelRoot so navigation/avoidance/commands work
    """
    from people.scripts.utils import Utils as PeopleUtils

    people_spawned = []
    pedestrians = pedestrian_cfg.get("pedestrians", [])

    # optional: write a commands file from YAML commands list so CharacterBehavior will pick up commands
    command_lines = []
    for ped in pedestrians:
        name = ped.get("name")
        # Support both new 'commands' and legacy 'target_position' formats
        commands_list = ped.get("commands") or []
        for cmd in commands_list:
            if isinstance(cmd, dict):
                # New format: {action: "GoTo", coord: [x,y,z], rotation: "_"}
                action = cmd.get("action", "").strip()
                if action == "GoTo":
                    coord = cmd.get("coord", [])
                    rotation = cmd.get("rotation", "_")
                    if len(coord) >= 3:
                        command_lines.append(f"{name} GoTo {float(coord[0])} {float(coord[1])} {float(coord[2])} {rotation}")
                elif action in ["Idle", "LookAround", "Sit"]:
                    # Simple actions without parameters
                    command_lines.append(f"{name} {action}")
                else:
                    carb.log_warn(f"Unknown action '{action}' for {name}")

    # if there are commands, write a local command file and point PeopleSettings.COMMAND_FILE_PATH at it
    if command_lines:
        cmd_path_local = os.path.abspath(os.path.join(os.getcwd(), "config", "people_command_file.txt"))
        os.makedirs(os.path.dirname(cmd_path_local), exist_ok=True)
        with open(cmd_path_local, "w") as f:
            f.write("\n".join(command_lines))
        cmd_uri = f"file://{cmd_path_local}"
        omni.kit.commands.execute("ChangeSetting", path=PeopleSettings.COMMAND_FILE_PATH, value=cmd_uri)
        carb.log_info(f"Wrote people command file -> {cmd_uri}")

    # spawn each character USD under the CHARACTER_PRIM_PATH
    characters_root = carb.settings.get_settings().get(PeopleSettings.CHARACTER_PRIM_PATH) or "/World/Characters"
    stage = omni.usd.get_context().get_stage()

    for ped_cfg in pedestrians:
        try:
            name = ped_cfg.get("name", "pedestrian_0")
            character = ped_cfg.get("character")
            pos = ped_cfg.get("position", [0.0, 0.0, 0.1])
            yaw = float(ped_cfg.get("yaw", 0.0))

            usd_path = _get_character_usd_path(character) if character else None
            if not usd_path:
                carb.log_warn(f"Character USD for '{character}' not found; attempting fallback list")
                # list available characters from the configured assets root
                assets_root = carb.settings.get_settings().get(PeopleSettings.CHARACTER_ASSETS_PATH) or get_assets_root_path_safe()
                assets_root = os.path.join(assets_root, "Isaac/People/Characters") if "Isaac/People/Characters" not in assets_root else assets_root
                try:
                    result, listing = omni.client.list(assets_root)
                    if result == omni.client.Result.OK and listing:
                        character = listing[0].relative_path
                        usd_path = _get_character_usd_path(character)
                except Exception:
                    pass

            if not usd_path:
                carb.log_error(f"No USD found for character '{character}' — skipping spawn for {name}")
                continue

            prim_path = f"{characters_root}/{name}"
            add_usd_reference(stage, prim_path, usd_path)
            set_xform_pose(stage, prim_path, pos, [1.0, 0.0, 0.0, 0.0])
            carb.log_info(f"Spawned character prim {prim_path} -> {usd_path}")

            # attach CharacterBehavior script to SkelRoot
            skelroot_prim_path = None
            root_prim = stage.GetPrimAtPath(characters_root)
            if root_prim and root_prim.IsValid():
                for child in root_prim.GetChildren():
                    if child.GetName() == name:
                        for p in Usd.PrimRange(child):
                            if p.GetTypeName() == "SkelRoot":
                                skelroot_prim_path = p.GetPath()
                                break
                        if skelroot_prim_path:
                            break

            # fallback: search whole stage by prim name
            if not skelroot_prim_path:
                for prim in Usd.PrimRange(stage.GetDefaultPrim()):
                    if prim.GetName() == name:
                        for p in Usd.PrimRange(prim):
                            if p.GetTypeName() == "SkelRoot":
                                skelroot_prim_path = p.GetPath()
                                break
                        if skelroot_prim_path:
                            break

            if skelroot_prim_path:
                ext_base = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module("omni.anim.people")
                script_path = ext_base + "/omni/anim/people/scripts/character_behavior.py"
                omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[Sdf.Path(skelroot_prim_path)])
                attr = stage.GetPrimAtPath(skelroot_prim_path).GetAttribute("omni:scripting:scripts")
                script_list = attr.Get() or []
                if script_path not in script_list:
                    script_list.append(script_path)
                    attr.Set(script_list)
                carb.log_info(f"Attached CharacterBehavior to {skelroot_prim_path}")
                people_spawned.append({"name": name, "prim_path": prim_path, "skelroot_path": str(skelroot_prim_path)})
            else:
                carb.log_warn(f"SkelRoot for {name} not found — CharacterBehavior not attached")
                people_spawned.append({"name": name, "prim_path": prim_path})

        except Exception as e:
            carb.log_error(f"Failed to spawn pedestrian {ped_cfg.get('name', 'unknown')}: {e}")
            continue

    # debug: list managed characters known to omni.anim.people
    try:
        from people.scripts.global_character_position_manager import GlobalCharacterPositionManager
        mgr = GlobalCharacterPositionManager.get_instance()
        carb.log_info(f"GlobalCharacterPositionManager entries after spawn: {list(mgr.get_all_managed_characters())}")
    except Exception:
        pass

    return people_spawned

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

    try:
        from people.python_ext import add_dynamic_obstacle_behavior_script
        base_link_prim = f"{robot_prim_path}/{base_frame_id}"
        add_dynamic_obstacle_behavior_script(base_link_prim)
        carb.log_info(f"Attached DynamicObstacle to {base_link_prim}")
    except Exception as e:
        carb.log_warn(f"Could not attach DynamicObstacle: {e}")
    robot_params = arena_simulation_setup.entities.robot.Robot(model_name).model_params  # :contentReference[oaicite:1]{index=1}

    #Jackal OmniGraph
    j = Jackal(name=name, prim_path=robot_prim_path)

    j.control_and_publish_joint_states()
    j.publish_odom_and_tf()

    # Attach DynamicObstacle so omni.anim.people sees the robot as a dynamic obstacle
    try:
        from people.python_ext import add_dynamic_obstacle_behavior_script
        add_dynamic_obstacle_behavior_script(base_link_prim)
        carb.log_info(f"Attached DynamicObstacle to {base_link_prim}")
    except Exception as e:
        carb.log_warn(f"Could not attach DynamicObstacle to {base_link_prim}: {e}")
 
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
        lidar_name = lidar_cfg.get("name", "lidar")
        lidar_parent_prim = lidar_cfg.get("parent_prim", base_link_prim)
        lidar = lidar_setup(lidar_parent_prim, lidar_name)

        publish_lidar(
            prim_path=robot_prim_path,
            lidar=lidar,
            topic_scan=lidar_cfg.get("topic_scan", "scan"),
            topic_pointcloud=lidar_cfg.get("topic_pointcloud", "points"), 
            frame_id=lidar_cfg.get("frame_id", "base_link"),
            context_domain_id=ros_domain_id,
        )

    # --- IMU ---
    imu_cfg = sensors_cfg.get("imu", {}) or {}
    if imu_cfg.get("enabled", True):
        imu_name = imu_cfg.get("name", "imu")
        imu_parent_prim = imu_cfg.get("parent_prim", base_link_prim)
        imu = imu_setup(imu_parent_prim, imu_name)
        # sensors.py has initialize() commented out -> do it here to be safe
        try:
            imu.initialize()
        except Exception:
            pass

        publish_imu(
            prim_path=robot_prim_path,
            link=imu_cfg.get("link", base_frame_id),
            imu=imu,
            topic=imu_cfg.get("topic", "imu"),
            frame_id=imu_cfg.get("frame_id", "base_link"),
            context_domain_id=ros_domain_id,
            debug_print=bool(imu_cfg.get("debug_print", False)),
        )

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
    parser.add_argument("--pedestrians", default=None, help="path to pedestrians.yaml (optional)")
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
    sim_context.reset()

    if pedestrian_cfg.get("pedestrians"):
        # optional persistent settings from YAML
        assets_root = pedestrian_cfg.get("assets_root")
        if assets_root:
            omni.kit.commands.execute("ChangeSetting", path=PeopleSettings.CHARACTER_ASSETS_PATH, value=assets_root)
            carb.log_info(f"Set PeopleSettings.CHARACTER_ASSETS_PATH -> {assets_root}")
        character_prim_path = pedestrian_cfg.get("character_prim_path")
        if character_prim_path:
            omni.kit.commands.execute("ChangeSetting", path=PeopleSettings.CHARACTER_PRIM_PATH, value=character_prim_path)
            carb.log_info(f"Set PeopleSettings.CHARACTER_PRIM_PATH -> {character_prim_path}")

        # configure navmesh + avoidance using people extension settings
        navmesh_cfg = pedestrian_cfg.get("navmesh", {})
        _configure_people_navmesh(navmesh_cfg)

        people = spawn_pedestrians(world, pedestrian_cfg)
        
        # Setup pedestrian ROS2 publisher (publishes positions/metadata to /people)
        # ped_skelroots = [p.get("skelroot_path") for p in people if p.get("skelroot_path")]
        # if ped_skelroots:
        #     publish_pedestrians(
        #         character_prim_paths=ped_skelroots,
        #         topic_name=pedestrian_cfg.get("ros2_topic", "/people"),
        #         context_domain_id=30,
        #         frame_id="world"
        #     )
        #     carb.log_info(f"Published {len(ped_skelroots)} pedestrians to ROS2")
        # else:
        #     carb.log_warn("No pedestrian SkelRoot paths found for ROS2 publishing")
    
    sim_context.play()
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

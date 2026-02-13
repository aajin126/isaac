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
from pedestrian.simulator.logic.people_manager import PeopleManager
from pedestrian.simulator.params import DEFAULT_WORLD_SETTINGS, SIMULATION_ENVIRONMENTS
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

# def load_urdf(robot_cfg: dict) -> str:
#     urdf_path = abs_path((robot_cfg.get("robot", {}) or {}).get("urdf_path", ""))
#     if not urdf_path or not os.path.exists(urdf_path):
#         raise FileNotFoundError(f"robot.urdf_path not found: {urdf_path}")
#     with open(urdf_path, "r") as f:
#         return f.read()
    
# def spawn_robot(robot_cfg: dict) -> str:
#     name = robot_cfg.get("name", "jackal")
#     robot_block = robot_cfg.get("robot", {})
#     #urdf_path = os.path.abspath(robot_block["urdf_path"])
#     prim_path = robot_block.get("prim_path", f"/World/Robots/{name}")

#     frames = robot_cfg.get("frames", {})
#     base_frame = frames.get("base_frame_id", "base_link")
#     odom_frame = frames.get("odom_frame_id", "odom")

#     pose = robot_block.get("pose", {})
#     pos = pose.get("position", [0.0, 0.0, 0.0])
#     quat = pose.get("orientation_quat_wxyz", [3.0, 0.0, 0.0, 0.0])

#     ros = robot_cfg.get("ros", {})
#     cmd_vel_topic = ros.get("cmd_vel_topic", f"/{name}/cmd_vel")

#     service_name = "/urdf_to_usd" 
#     service_type = "UrdfToUsd" 

#     # ros2 service call
#     req_yaml = f"""
#     name: "{name}"
#     robot_model: "{robot_cfg.get('model', name)}"
#     no_localization: false
#     base_frame: "{base_frame}"
#     odom_frame: "{odom_frame}"
#     cmd_vel_topic: "{cmd_vel_topic}"
#     pose:
#       position: {{x: {pos[0]}, y: {pos[1]}, z: {pos[2]}}}
#       orientation: {{w: {quat[0]}, x: {quat[1]}, y: {quat[2]}, z: {quat[3]}}}
#     """
#     subprocess.run(
#         ["ros2", "service", "call", service_name, service_type, req_yaml],
#         check=True,
#         text=True,
#     )

#     return prim_path

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
    extensions.enable_extension("omni.anim.graph.core")
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
def spawn_pedestrians(world, pedestrian_cfg: dict) -> list:
    """
    Spawn pedestrians based on configuration.
    
    Args:
        world: Isaac Sim world context
        pedestrian_cfg: Configuration dict with list of pedestrians
        
    Returns:
        list: List of spawned Person objects
    """
    from pedestrian.simulator.logic.people.person import Person
    people = []
    pedestrians = pedestrian_cfg.get("pedestrians", [])
    
    for ped_cfg in pedestrians:
        try:
            name = ped_cfg.get("name", "pedestrian_0")
            character = ped_cfg.get("character", "Female_1")
            pos = ped_cfg.get("position", [0.0, 0.0, 0.1])
            yaw = float(ped_cfg.get("yaw", 0.0))
            target_pos = ped_cfg.get("target_position", [pos])
            walk_speed = float(ped_cfg.get("walk_speed", 1.0))
            
            # Check if character exists
            try:
                Person.get_path_for_character_prim(character)
            except Exception as e:
                carb.log_warn(f"Character {character} not found for {name}, using default")
                available = Person.get_character_asset_list()
                if available:
                    character = available[0]
                    carb.log_info(f"Using {character} instead")
                else:
                    carb.log_error("No character assets available")
                    continue
            
            # Spawn person
            person = Person(
                world=world,
                stage_prefix=name,
                character_name=character,
                init_pos=pos,
                init_yaw=yaw,
            )
            
            # Set target position
            person.update_target_position(target_pos, walk_speed)
            people.append(person)
            carb.log_info(f"Spawned pedestrian: {name} (character: {character})")
            
        except Exception as e:
            carb.log_error(f"Failed to spawn pedestrian {ped_cfg.get('name', 'unknown')}: {e}")
            continue
    
    return people

# def build_robot_and_ros(robot_cfg: dict):
#     stage = omni.usd.get_context().get_stage()

#     name = robot_cfg.get("name", "jackal")
#     robot_block = robot_cfg.get("robot", {})
#     robot_usd = abs_path(robot_block.get("usd_path", ""))
#     robot_prim_path = robot_block.get("prim_path", f"/World/Robots/{name}")
#     base_prim_name = robot_block.get("base_prim", "base_link")

#     if not robot_usd or not os.path.exists(robot_usd):
#         raise FileNotFoundError(f"robot.usd not found: {robot_usd}")

#     # Reference the robot USD
#     add_usd_reference(stage, robot_prim_path, robot_usd)

#     # Set pose on robot root prim
#     pose = robot_block.get("pose", {})
#     pos = pose.get("position", [0.0, 0.0, 0.0])
#     quat = pose.get("orientation_quat_wxyz", [1.0, 0.0, 0.0, 0.0])
#     set_xform_pose(stage, robot_prim_path, pos, quat)

#     # door manager optional
#     if door_manager is not None:
#         try:
#             door_manager.add_robot(robot_prim_path)
#         except Exception as e:
#             carb.log_warn(f"door_manager.add_robot failed: {e}")

#     simulation_app.update()

#     # ROS graph setup
#     ros = robot_cfg.get("ros", {})
#     tf_prefix = ros.get("tf_prefix", name)
#     cmd_vel_topic = ros.get("cmd_vel_topic", f"/{name}/cmd_vel")
#     joint_states_topic = ros.get("joint_states_topic", f"/{name}/joint_states")

#     publish_clock = bool(ros.get("publish_clock", True))
#     publish_tf_flag = bool(ros.get("publish_tf", True))
#     publish_odom_flag = bool(ros.get("publish_odom", True))
#     publish_js_flag = bool(ros.get("publish_joint_states", True))

#     if publish_clock:
#         PublishTime("/World/publish_time")

#     base_path = f"{robot_prim_path}/{base_prim_name}"

#     # Odom (raw tf tree) – publish if you want IsaacSim side odom
#     if publish_odom_flag:
#         frames = robot_cfg.get("frames", {})
#         odom_frame_id = frames.get("odom_frame_id", "odom")
#         base_frame_id = frames.get("base_frame_id", "base_link")
#         odom(
#             graph_path=f"{robot_prim_path}/odom_graph",
#             prim_path=base_path,
#             base_frame_id=base_frame_id if not tf_prefix else f"{tf_prefix}/{base_frame_id}",
#             odom_frame_id=odom_frame_id if not tf_prefix else f"{tf_prefix}/{odom_frame_id}",
#             map_frame_id="map",
#         )

#     # TF tree – turn OFF if your ROS localization publishes TF already
#     if publish_tf_flag:
#         tf(
#             graph_path=f"{robot_prim_path}/tf_graph",
#             prim_path=base_path,
#             tf_prefix=tf_prefix,
#         )

#     # Joint states
#     if publish_js_flag:
#         joint_states(
#             graph_path=f"{robot_prim_path}/joint_states_graph",
#             prim_path=base_path,
#             joint_states_topic=joint_states_topic,
#         )

#     # cmd_vel -> diff drive
#     diff = robot_cfg.get("diff_drive", {})
#     if bool(diff.get("enabled", True)):
#         differential(
#             graph_path=f"{robot_prim_path}/diff_drive_graph",
#             prim_path=robot_prim_path,
#             cmd_vel_topic=cmd_vel_topic,
#             joint_names=diff.get("wheel_joints", ["wheel_left_joint", "wheel_right_joint"]),
#             wheel_distance=float(diff.get("wheel_distance", 0.36)),
#             wheel_radius=float(diff.get("wheel_radius", 0.098)),
#             max_linear_speed=float(diff.get("max_linear_speed", 2.0)),
#             min_linear_speed=0.0,
#             max_angular_speed=float(diff.get("max_angular_speed", 4.0)),
#             min_angular_speed=0.0,
#         )

#     simulation_app.update()

#     urdf_xml_string = load_urdf(robot_cfg)
#     s = Sensors(prim_path=robot_prim_path, base_topic=ros.get("base_topic", ""))
#     s.parse_gazebo(urdf_xml_string)

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
    from pedestrian.simulator.logic.people.person import Person
    world_cfg = load_yaml(args.world)
    robot_cfg = load_yaml(args.robot)
    pedestrian_cfg = load_yaml(args.pedestrians) if args.pedestrians else {}
    
    # Simulation context
    stage_units = float(world_cfg.get("stage_units_in_meters", 1.0))
    sim_context = SimulationContext(stage_units_in_meters=stage_units)
    world = World(stage_units_in_meters=stage_units)
    
    # Build
    build_world(world_cfg)
    build_robot_and_ros(robot_cfg)
    sim_context.reset()
    # Initialize PeopleManager for NavMesh (optional pedestrians)
    if pedestrian_cfg.get("pedestrians"):
        from pedestrian.simulator.logic.people.person import Person
        people_manager = PeopleManager()
        people = spawn_pedestrians(sim_context, pedestrian_cfg)

        carb.log_info(f"Spawned {len(people)} pedestrians")
    else:
        carb.log_info("No pedestrians configured - skipping pedestrian support")

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

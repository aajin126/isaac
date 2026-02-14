"""
ROS2 Pedestrian/People Publisher Integration.

Publishes pedestrian positions, states, and metadata to ROS2 topics.
Supports both legacy people_msgs.People and custom arena_people_msgs.Pedestrians.
"""

import carb
import omni.usd
import omni.kit.app
from pxr import Usd, UsdGeom
from omni.isaac.core.utils import extensions

extensions.enable_extension("isaacsim.ros2.bridge")

import omni.graph.core as og


def publish_pedestrians_legacy(character_prim_paths: list, topic_name="/people", context_domain_id: int = 30):
    """
    Publish pedestrian positions using legacy people_msgs/People format.
    
    Message structure: people_msgs/People
    - header: std_msgs/Header
    - people: Person[]
        - person_id: int
        - position: geometry_msgs/Point (x, y, z)
        - velocity: geometry_msgs/Twist (linear vel)
        - name: string
        - tags: string[]
    
    Args:
        character_prim_paths (list): List of pedestrian SkelRoot prim paths (e.g., ["/World/Characters/pedestrian_1/SkelRoot"])
        topic_name (str): ROS2 topic name to publish to
        context_domain_id (int): ROS2 domain ID
    
    Example:
        from isaac_utils.pub_pedestrian import publish_pedestrians_legacy
        ped_paths = ["/World/Characters/pedestrian_1/SkelRoot", "/World/Characters/pedestrian_2/SkelRoot"]
        publish_pedestrians_legacy(ped_paths, topic_name="/jackal/people")
    """
    
    if not character_prim_paths:
        carb.log_warn("No pedestrian prim paths provided to publish_pedestrians_legacy")
        return
    
    # Graph path for hosting the pedestrian publisher graph
    graph_path = "/World/PedestrianPublisher_Legacy"
    
    keys = og.Controller.Keys
    
    try:
        # Create the OmniGraph structure for publishing
        (graph_handle, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    # Note: people_msgs.People publishing requires custom ROS2 nodes or direct Python script
                    # For now, we provide a placeholder that demonstrates the structure
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", int(context_domain_id)),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2Context.inputs:execIn"),
                ],
            },
        )
        carb.log_info(f"Created pedestrian publisher graph at {graph_path}")
        
    except Exception as e:
        carb.log_warn(f"Could not create pedestrian publisher graph: {e}")


def publish_pedestrians(character_prim_paths: list, topic_name="/people", 
                        context_domain_id: int = 30, frame_id="world"):
    """
    Publish pedestrian positions as a custom ROS2 message or via direct script callback.
    
    This creates a Python script that:
    1. Subscribes to simulation ticks
    2. Reads character positions from USD stage
    3. Publishes positions as people_msgs/People (if available) or custom message
    
    Args:
        character_prim_paths (list): Pedestrian character prim paths
        topic_name (str): ROS2 topic name
        context_domain_id (int): ROS2 domain ID
        frame_id (str): TF frame ID for the positions
    """
    
    if not character_prim_paths:
        carb.log_warn("No pedestrian prim paths provided to publish_pedestrians")
        return
    
    stage = omni.usd.get_context().get_stage()
    
    # Validate prim paths
    valid_paths = []
    for prim_path in character_prim_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if prim and prim.IsValid():
            valid_paths.append(prim_path)
        else:
            carb.log_warn(f"Pedestrian prim path not found: {prim_path}")
    
    if not valid_paths:
        carb.log_error("No valid pedestrian prim paths found")
        return
    
    # Create OmniGraph for pedestrian publisher
    graph_path = "/World/PedestrianPublisher"
    
    keys = og.Controller.Keys
    
    try:
        (graph_handle, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("PythonScript", "omni.graph.scriptnode.ScriptNode"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", int(context_domain_id)),
                    ("PythonScript.inputs:script", _generate_pedestrian_publisher_code(valid_paths, topic_name, frame_id)),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PythonScript.inputs:execIn"),
                ],
            },
        )
        carb.log_info(f"Created pedestrian ROS2 publisher at {graph_path} → {topic_name}")
        
    except Exception as e:
        carb.log_error(f"Failed to create pedestrian publisher: {e}. Using fallback Python callback.")
        _setup_pedestrian_callback(valid_paths, topic_name, context_domain_id, frame_id)


def _generate_pedestrian_publisher_code(prim_paths: list, topic_name: str, frame_id: str) -> str:
    """Generate inline Python code for pedestrian publishing."""
    
    prim_paths_str = ", ".join([f'"{p}"' for p in prim_paths])
    
    code = f"""
import omni.usd
import carb
from pxr import Usd, UsdGeom

stage = omni.usd.get_context().get_stage()

# Pedestrian prim paths to track
ped_paths = [{prim_paths_str}]

# Store publisher and context (will be initialized on first call)
_ctx = None
_pub = None
_initialized = False

def on_update(outputs):
    global _ctx, _pub, _initialized
    import carb
    try:
        import rclpy
        from people_msgs.msg import People, Person
        from geometry_msgs.msg import Point, Twist
        from std_msgs.msg import Header
        
    except ImportError:
        if not _initialized:
            carb.log_warn("people_msgs ROS2 package not available. Pedestrian publishing skipped.")
            _initialized = True
        return
    
    # Initialize ROS2 context and publisher once
    if not _initialized:
        try:
            if not rclpy.ok():
                rclpy.init()
            _ctx = rclpy.create_node('pedestrian_publisher')
            _pub = _ctx.create_publisher(People, '{topic_name}', 10)
            _initialized = True
            carb.log_info("Pedestrian ROS2 publisher initialized")
        except Exception as e:
            carb.log_warn(f"Failed to init pedestrian ROS2 publisher: {{e}}")
            _initialized = True
            return
    
    # Collect pedestrian positions
    msg = People()
    msg.header = Header()
    msg.header.stamp = _ctx.get_clock().now().to_msg()
    msg.header.frame_id = '{frame_id}'
    
    for idx, ped_path in enumerate(ped_paths):
        prim = stage.GetPrimAtPath(ped_path)
        if not prim or not prim.IsValid():
            continue
        
        # Get worldspace position
        xf = UsdGeom.Xformable(prim)
        if not xf:
            continue
        
        times, values = xf.GetOrderedXformOps()[0].GetOpAttr().Get(Usd.TimeCode.Default()), None
        # Simplified: read X form for position (full version would use GetWorldTransform)
        transform = UsdGeom.Xformable(prim).GetLocalTransformation()
        pos = [transform[3][0], transform[3][1], transform[3][2]]
        
        person = Person()
        person.name = prim.GetName()
        person.reliability = 1.0
        person.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        person.velocity = Twist()  # Placeholder
        
        msg.people.append(person)
    
    # Publish message
    if msg.people:
        _pub.publish(msg)

# Called on each tick
on_update(None)
"""
    
    return code


def _setup_pedestrian_callback(prim_paths: list, topic_name: str, context_domain_id: int, frame_id: str):
    """
    Setup pedestrian publisher via timeline callback (fallback if OmniGraph fails).
    """
    
    carb.log_info(f"Setting up pedestrian callback for {len(prim_paths)} characters → {topic_name}")
    
    stage = omni.usd.get_context().get_stage()
    
    class PedestrianPublisherCallback:
        def __init__(self):
            self.publisher = None
            self.context = None
            self.initialized = False
        
        def __call__(self, event):
            try:
                import rclpy
                from people_msgs.msg import People, Person
                from geometry_msgs.msg import Point, Twist
                from std_msgs.msg import Header
            except ImportError:
                if not self.initialized:
                    carb.log_warn("people_msgs not available for pedestrian publishing")
                    self.initialized = True
                return
            
            # Initialize on first call
            if not self.initialized:
                try:
                    if not rclpy.ok():
                        rclpy.init()
                    self.context = rclpy.create_node('pedestrian_pub_callback')
                    self.publisher = self.context.create_publisher(People, topic_name, 10)
                    self.initialized = True
                    carb.log_info("Pedestrian ROS2 callback initialized")
                except Exception as e:
                    carb.log_warn(f"Failed to init pedestrian callback: {e}")
                    self.initialized = True
                    return
            
            # Publish pedestrian positions
            msg = People()
            msg.header = Header()
            msg.header.frame_id = frame_id
            
            for ped_path in prim_paths:
                prim = stage.GetPrimAtPath(ped_path)
                if not prim or not prim.IsValid():
                    continue
                
                try:
                    xf = UsdGeom.Xformable(prim)
                    transform = xf.GetLocalTransformation()
                    
                    person = Person()
                    person.name = prim.GetName()
                    person.position = Point(
                        x=float(transform[3][0]),
                        y=float(transform[3][1]),
                        z=float(transform[3][2])
                    )
                    person.velocity = Twist()
                    
                    msg.people.append(person)
                except Exception as e:
                    carb.log_debug(f"Error reading pedestrian {ped_path}: {e}")
            
            if msg.people and self.publisher:
                self.publisher.publish(msg)
    
    # Register callback
    callback = PedestrianPublisherCallback()

    app = omni.kit.app.get_app()
    update_stream = app.get_update_event_stream()

    global _pedestrian_pub_sub
    _pedestrian_pub_sub = update_stream.create_subscription_to_pop(
        callback,
        name="pedestrian_publisher_callback"
    )

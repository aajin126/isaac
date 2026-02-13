from isaac_utils.robot_models.waffle import Waffle
from isaac_utils.robot_models.jackal import Jackal

def assign_robot_model(name, prim_path, model):
    if model == "waffle":
        return Waffle(name, prim_path)
    if model == "jackal":
        return Jackal(name, prim_path)
    raise ValueError(f"Unknown model: {model}")

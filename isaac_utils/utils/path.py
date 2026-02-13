import os
import re


def sanitize_path_component(component: str) -> str:
    if not re.match(r'^[a-zA-Z_]', component):
        return f'_{component}'
    return component


def world_path(*path: str) -> str:
    return os.path.join('/World', *map(sanitize_path_component, path))


def pedestrian_path(name: str) -> str:
    """
    Returns the path to the pedestrian directory in the world.
    """
    return world_path('pedestrians', name)

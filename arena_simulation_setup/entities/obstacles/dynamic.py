import os

from arena_simulation_setup import Interface, ass_dir
from arena_simulation_setup.utils.models.model_loader import ModelLoader


class DynamicObstacle(Interface(os.path.join(ass_dir, 'entities', 'obstacles', 'dynamic'))):
    ...


loader = ModelLoader(DynamicObstacle.base_dir())

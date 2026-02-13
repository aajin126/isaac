import os
import typing

import yaml

from arena_simulation_setup import Interface, ass_dir
from arena_simulation_setup.utils.models.model_loader import ModelLoader



class ModelParams(dict[str, typing.Any]):
    @classmethod
    def from_yaml(cls, path: str) -> 'ModelParams':
        with open(path) as f:
            return cls(yaml.safe_load(f))

    @property
    def base_frame(self) -> str:
        return self.get('robot_base_frame', 'base_link')

    @property
    def odom_frame(self) -> str:
        return self.get('robot_odom_frame', 'odom')

    @property
    def z_offset(self) -> float:
        return self.get('z_offset', 0.0)


class Robot(Interface(os.path.join(ass_dir, 'entities', 'robots'))):

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._cached_params = None

    @property
    def model_params(self) -> ModelParams:
        if self._cached_params is None:
            self._cached_params = ModelParams.from_yaml(os.path.join(self.path, 'model_params.yaml'))
        return self._cached_params

    @property
    def mappings(self) -> str:
        return os.path.join(
            self.path,
            'mappings.yaml'
        )

    @property
    def control(self) -> dict:
        with open(os.path.join(self.path, 'control.yaml')) as f:
            return yaml.safe_load(f)


loader = ModelLoader(Robot.base_dir())

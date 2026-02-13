from __future__ import annotations

import itertools
import os

import attrs
import yaml

from arena_simulation_setup import Interface, ass_dir
from arena_simulation_setup.shared import DynamicObstacle, Obstacle, Pose


@attrs.define
class RobotGoal:
    start: Pose
    goal: Pose

    @classmethod
    def parse(cls, obj: dict) -> RobotGoal:
        return cls(
            start=Pose.parse(obj.get("start", [])),
            goal=Pose.parse(obj.get("goal", [])),
        )


@attrs.define
class Scenario:
    static: list[Obstacle]
    dynamic: list[DynamicObstacle]
    robots: list[RobotGoal]


class _Map:
    def __init__(self, path: str):
        self._path = path

    @property
    def path(self) -> str:
        return self._path

    @property
    def map_yaml(self) -> str:
        return os.path.join(self.path, 'map.yaml')

    @property
    def obstacles(self) -> str:
        return os.path.join(self.path, 'obstacles.yaml')

    @property
    def walls(self) -> str:
        return os.path.join(self.path, 'walls.yaml')

    @property
    def zones(self) -> str:
        return os.path.join(self.path, 'zones.yaml')


class World(Interface(os.path.join(ass_dir, 'worlds'))):

    @classmethod
    def list(cls) -> list[str]:
        return ['.generated'] + super().list()

    @property
    def scenario(self):
        class _Scenario(Interface(os.path.join(self.path, 'scenarios'))):
            def load(self) -> Scenario:
                with open(self.path, 'r') as f:
                    scenario = yaml.safe_load(f)

                return Scenario(
                    static=[
                        Obstacle.parse(
                            obs,
                        )
                        for obs
                        in itertools.chain(
                            scenario.get("obstacles", {}).get("static", []),
                            scenario.get("obstacles", {}).get("interactive", [])
                        )
                    ],
                    dynamic=[
                        DynamicObstacle.parse(obs)
                        for obs
                        in scenario.get("obstacles", {}).get("dynamic", [])
                    ],
                    robots=[
                        RobotGoal.parse(robot)
                        for robot
                        in scenario.get("robots", [])
                    ]
                )
        return _Scenario

    @property
    def map(self):
        return _Map(os.path.join(self.path, 'map'))

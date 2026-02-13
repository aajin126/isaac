import re
import typing

import attrs

from arena_simulation_setup.entities.obstacles.dynamic import \
    loader as DynamicObstacleLoader
from arena_simulation_setup.entities.obstacles.static import \
    loader as ObstacleLoader
from arena_simulation_setup.entities.robot import loader as RobotLoader
from arena_simulation_setup.utils.models import ModelWrapper
from arena_simulation_setup.utils.models.model_loader import ModelLoader

from .utils.geometry import Pose, Position


def model_parse(parser: ModelLoader, *, overrides: typing.Iterable[ModelLoader] = ()) -> typing.Callable[[typing.Any], ModelWrapper]:
    def validator(v: str | ModelWrapper) -> ModelWrapper:
        if isinstance(v, ModelWrapper):
            if any(v.loader_matches(overridee) for overridee in overrides):
                return parser.bind(v.name)
            return v
        return parser.bind(v)
    return validator


@attrs.define
class Wall:
    Start: Position
    End: Position
    height: float = attrs.field(converter=float, default=2.)
    texture_material: str = ''  # not implemented

    @classmethod
    def parse(cls, obj: list) -> "Wall":
        kwargs = {}
        if len(obj) > 2 and isinstance(obj[2], dict):
            kwargs = obj[2]
        return cls(
            **kwargs,
            Start=Position(x=obj[0][0], y=obj[0][1]),
            End=Position(x=obj[1][0], y=obj[1][1]),
        )


@attrs.define
class Entity:
    pose: Pose
    name: str = attrs.field(converter=lambda s: Entity.sanitize_name(str(s)))
    model: ModelWrapper
    extra: dict = attrs.field(factory=dict, kw_only=True)

    def asdict(self, expand_extra: bool = True) -> dict:
        if expand_extra:
            return {
                **attrs.asdict(self, filter=lambda a, v: a.name != 'extra'),
                **self.extra,
            }
        return attrs.asdict(self)

    @classmethod
    def sanitize_name(cls, name: str) -> str:
        return re.sub('[^A-Za-z0-9_]', '_', name)


@attrs.define
class Obstacle(Entity):
    model: ModelWrapper = attrs.field(converter=model_parse(ObstacleLoader))

    @classmethod
    def parse(cls, obj: dict) -> "Obstacle":
        name = str(obj.get("name", ""))
        pose = Pose.parse(obj.get("pos", (0, 0, 0)))
        model = str(obj.get("model", ""))

        return cls(
            name=name,
            pose=pose,
            model=model,
            extra=obj,
        )


@attrs.define
class DynamicObstacle(Obstacle):
    model: ModelWrapper = attrs.field(converter=model_parse(DynamicObstacleLoader, overrides=(ObstacleLoader,)))
    waypoints: list[Position]

    @classmethod
    def parse(cls, obj: dict) -> "DynamicObstacle":

        base = Obstacle.parse(obj)
        waypoints = [
            Position(*waypoint)
            for waypoint
            in obj.get("waypoints", [])
        ]

        return cls(
            **attrs.asdict(base, recurse=False),
            waypoints=waypoints,
        )


@attrs.define
class Robot(Entity):
    model: ModelWrapper = attrs.field(converter=model_parse(RobotLoader))

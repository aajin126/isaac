import abc
import enum
import typing

import pydantic
import rclpy.node
import std_srvs.srv

from .utils import GeneratedWorld, Polygon  # noqa


class WorldGeneratorType(enum.Enum):
    """
    Enum for world generator types.
    """
    EMPTY = "empty"
    HALLWAY = "hallway"


class _BaseConfiguration(pydantic.BaseModel):
    width: float = 15.0
    height: float = 15.0
    resolution: float = 0.05


class _WorldGeneratorImpl(abc.ABC):
    """
    Abstract base class for world generators.
    """

    def __init__(self, configuration: dict) -> None:
        super().__init__()
        self.configure(configuration)

    @abc.abstractmethod
    def configure(self, configuration: dict):
        ...

    @abc.abstractmethod
    def compute(self) -> GeneratedWorld:
        ...


class _WorldGenerator:
    __registry: typing.ClassVar[dict[WorldGeneratorType, typing.Type[_WorldGeneratorImpl]]] = {}
    _active: _WorldGeneratorImpl

    @classmethod
    def register(cls, name: WorldGeneratorType):
        def wrap(impl: typing.Type[_WorldGeneratorImpl]):
            cls.__registry[name] = impl
            return impl
        return wrap

    def compute(self) -> GeneratedWorld:
        return self._active.compute()

    def update_generator(self, generator: WorldGeneratorType, configuration: dict):
        self._active: _WorldGeneratorImpl = self.__registry[generator](configuration)

    def __init__(self, generator: WorldGeneratorType, configuration: dict):
        self.update_generator(generator, configuration)


class WorldGeneratorROS(_WorldGenerator, rclpy.node.Node):
    def _cb_generate(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response) -> std_srvs.srv.Trigger.Response:
        try:
            self.compute().save_to('.generated')
            response.success = True
        except BaseException as e:
            response.success = False
            response.message = repr(e)

        return response

    def __init__(self, name: str, generator: WorldGeneratorType, configuration: dict):
        rclpy.node.Node.__init__(self, name)
        _WorldGenerator.__init__(self, generator, configuration)

        self.set_up_services()

    def set_up_services(self):
        self.create_service(std_srvs.srv.Trigger, 'generate_world', self._cb_generate)

import os
import typing

import ament_index_python.packages

ass_dir = "/home/ewhaglab/isaac/arena_simulation_setup"
ab_dir = "/home/ewhaglab/isaac/arena_simulation_setup"


class _InterfaceBase:
    _base_dir: typing.ClassVar[str]
    _name: str

    @classmethod
    def _listdir(cls, path: str) -> list[str]:
        return list(sorted(f for f in os.listdir(path) if not f.startswith('.')))

    @classmethod
    def base_dir(cls) -> str:
        return cls._base_dir

    @classmethod
    def list(cls) -> list[str]:
        return cls._listdir(cls._base_dir)

    def __init__(self, name: str) -> None:
        self._name = name

    @property
    def path(self) -> str:
        return os.path.join(
            self._base_dir,
            self._name,
        )


def Interface(base_dir: str):
    class _Interface(_InterfaceBase):
        ...

    _Interface._base_dir = base_dir

    return _Interface

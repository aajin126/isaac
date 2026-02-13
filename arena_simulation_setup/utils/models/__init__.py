from __future__ import annotations

import abc
import enum
import functools
import os
from collections.abc import Callable, Collection, Set
from typing import Optional, Type, overload

import attrs

# TODO deprecate this in favor of Model.EMPTY


def _EMPTY_LOADER(*_, **__) -> Model:

    return Model(
        type=ModelType.UNKNOWN, name="", description="", path=""
    )


EMPTY_LOADER = _EMPTY_LOADER


class ModelType(enum.Enum):
    UNKNOWN = ""
    URDF = "urdf"
    SDF = "sdf"
    YAML = "yaml"
    USD = "usd"


@attrs.frozen()
class Model:
    type: ModelType
    name: str
    description: str
    path: str

    @property
    def mapper(self) -> Callable[[Model], Model]:
        """
        Returns a (Model)->Model mapper that simply returns this model
        """
        return lambda m: self

    def replace(self, **kwargs) -> Model:
        """
        Wrapper for attrs.evolve
        **kwargs: properties to replace
        """
        return attrs.evolve(self, **kwargs)


class ITF_ModelLoader(abc.ABC):
    @classmethod
    @abc.abstractmethod
    def load(cls, model_dir: str, model: str, loader_args: dict) -> Model | None:
        return None

    @classmethod
    @abc.abstractmethod
    def convertable(cls) -> Collection[ModelType]:
        """
        return collection of model types convertable
        """
        return ()

    @classmethod
    @abc.abstractmethod
    def convert(cls, model_dir: str, model: Model, loader_args: dict) -> Model | None:
        return None


class ModelWrapper:
    _get: Callable[[Collection[ModelType], dict], Model]
    _name: str
    _override: dict[ModelType, tuple[bool, Callable[..., Model]]]
    _loader: object

    def loader_matches(self, loader: object) -> bool:
        return self._loader is loader

    def __repr__(self) -> str:
        return f"ModelWrapper(name={self.name}, loader={self._loader})"

    def __init__(
        self,
        name: str,
        callback: Callable[[Collection[ModelType], dict], Model] | None = None,
        loader: ITF_ModelLoader | None = None,
    ):
        """
        Create new ModelWrapper
        @name: Name of the ModelWrapper (should match the underlying Models)
        """
        self._loader = loader
        if self._loader is None:
            self._loader = object()
        if callback is None:
            callback = EMPTY_LOADER
        self._name = name
        self._get = callback
        self._override = {}

    def clone(self) -> ModelWrapper:
        """
        Clone (shallow copy) this ModelWrapper instance
        """
        clone = ModelWrapper(self.name, self._get, loader=self._loader)
        clone._override = self._override
        return clone

    def override(
        self,
        model_type: ModelType,
        override: Callable[[Model], Model],
        noload: bool = False,
        name: Optional[str] = None,
    ) -> ModelWrapper:
        """
        Create new ModelWrapper with an overridden ModelType callback
        @model_type: Which ModelType to override
        @override: Mapping function (Model)->Model which replaces the loaded model with a new one
        @noload: (default: False) If True, indicates that the original Model is not used by the override function and a dummy Model can be passed to it instead
        @name: (optional) If set, overrides name of ModelWrapper
        """
        clone = self.clone()
        clone._loader = object()
        clone._override = {**self._override, model_type: (noload, override)}

        if name is not None:
            clone._name = name

        return clone

    @overload
    def get(
        self,
        only: ModelType,
        *,
        loader_args: dict | None = None,
        **kwargs
    ) -> Model:
        """
            load specific model
            @only: single accepted ModelType
        """

    @overload
    def get(
        self,
        only: Collection[ModelType],
        *,
        loader_args: dict | None = None,
        **kwargs
    ) -> Model:
        """
            load specific model from collection
            @only: collection of acceptable ModelTypes
        """

    @overload
    def get(
        self,
        *,
        loader_args: dict | None = None,
        **kwargs
    ) -> Model:
        """
            load any available model
        """

    def get(
        self,
        only: ModelType | Collection[ModelType] | None = None,
        *,
        loader_args: dict | None = None,
        **kwargs,
    ) -> Model:
        if only is None:
            only = self._override.keys()

        if isinstance(only, ModelType):
            return self.get([only])

        if loader_args is None:
            loader_args = {}

        for model_type in only:
            if model_type in self._override:
                noload, mapper = self._override[model_type]

                if noload == True:
                    return mapper(EMPTY_LOADER())

                return mapper(self._get([model_type], loader_args), **kwargs)

        return self._get(only, loader_args)

    @property
    def name(self) -> str:
        """
        get name
        """
        return self._name

    @staticmethod
    def Constant(name: str, models: dict[ModelType, Model]) -> ModelWrapper:
        """
        Create new ModelWrapper from a dict of already existing models
        @name: name of model
        @models: dictionary of ModelType->Model mappings
        """

        def get(only: Collection[ModelType], loader_args: dict) -> Model:
            if not len(only):
                only = list(models.keys())

            for model_type in only:
                if model_type in models:
                    return models[model_type]
            raise LookupError(
                f"no matching model found for {name} (available: {list(models.keys())}, requested: {list(only)})"
            )

        return ModelWrapper(name, get)

    @staticmethod
    def from_model(model: Model) -> ModelWrapper:
        """
        Create new ModelWrapper containing a single existing Model
        @model: Model to wrap
        """
        return ModelWrapper.Constant(
            name=model.name,
            models={model.type: model}
        )

    @staticmethod
    def EMPTY() -> ModelWrapper:
        wrapper = ModelWrapper("__EMPTY", EMPTY_LOADER)
        return wrapper


class _ModelLoader:

    _registry: dict[ModelType, Type[ITF_ModelLoader]] = {}
    _models: Set[str]

    @classmethod
    def model(cls, model_type: ModelType):
        def inner(loader: Type[ITF_ModelLoader]):
            cls._registry[model_type] = loader
        return inner

    _model_dir: str
    _cache: dict[tuple[ModelType, str], Model]

    def __init__(self, model_dir: str):
        self._model_dir = model_dir
        self._cache = dict()
        self._models = set()

        # # potentially expensive
        # rospy.logdebug(
        #     f"models in {os.path.basename(model_dir)}: {self.models}")

    @property
    def models(self) -> Set[str]:
        if not len(self._models):
            if os.path.isdir(self._model_dir):
                self._models = set(next(os.walk(self._model_dir))[1])
            else:
                # rospy.logwarn(
                # f"Model directory {self._model_dir} does not exist. No models
                # are provided.")
                self._models = set()

        return self._models

    def bind(self, model: str) -> ModelWrapper:
        return ModelWrapper(
            name=model,
            callback=functools.partial(self._load_safe, model),
            loader=self,
        )

    def _load(self, model: str, only: Collection[ModelType], loader_args: dict) -> Model | None:
        if not only:
            only = self._registry.keys()

        only = [t for t in only if t in self._registry]

        for model_type in only:  # cache pass
            if (model_type, model) in self._cache:
                return self._cache[(model_type, model)]

        for model_type in only:  # disk pass
            hit = self._registry[model_type].load(self._model_dir, model, loader_args)
            if hit is not None:
                self._cache[(model_type, model)] = hit
                return self._cache[(model_type, model)]

        for model_type in only:  # try to convert
            targets = self._registry[model_type].convertable()
            if not targets:
                continue
            match = self._load(model, targets, loader_args)
            if match is not None:
                converted = self._registry[model_type].convert(self._model_dir, match, loader_args)
                if converted is None:
                    continue

                self._cache[(model_type, model)] = converted
                return converted

        return None

    def _load_safe(self, model: str, only: Collection[ModelType], loader_args: dict) -> Model:
        loaded = self._load(model, only, loader_args)
        if loaded is not None:
            return loaded

        raise FileNotFoundError(f"no model {model} among {only} found in {self._model_dir} and could not be converted")

from . import (GeneratedWorld, Polygon, WorldGeneratorType, _BaseConfiguration,
               _WorldGenerator, _WorldGeneratorImpl)


@_WorldGenerator.register(WorldGeneratorType.EMPTY)
class WorldGeneratorEmpty(_WorldGeneratorImpl):

    class Configuration(_BaseConfiguration):
        ...

    config: Configuration

    def configure(self, configuration: dict):
        self.config = self.Configuration.model_validate(configuration)

    def compute(self) -> GeneratedWorld:
        return GeneratedWorld(
            rooms=[
                Polygon([(0, 0), (self.config.width, 0), (self.config.width, self.config.height), (0, self.config.height)])
            ],
            doors=[],
            resolution=self.config.resolution,
            width=self.config.width,
            height=self.config.height,
        )

import sys

from . import WorldGeneratorROS, WorldGeneratorType
from . import _WorldGenerator as WorldGenerator

from .empty import *  # noqa
from .hallway import *  # noqa


__all__ = ['WorldGenerator', 'WorldGeneratorType', 'WorldGeneratorROS']


def test_generate(out: str, name: str, config: dict):
    gen = WorldGenerator(WorldGeneratorType(name), config)
    gen.compute().save_to(out)


def main(argv=sys.argv):
    import json
    import os

    if len(argv) == 3:
        test_generate(argv[1], argv[2], {})
    elif len(argv) == 4:
        test_generate(argv[1], argv[2], json.loads(argv[3]))
    else:
        print(f'usage: {os.path.basename(__file__)} <world_name> <generator> [<config>]')
        sys.exit(1)


if __name__ == '__main__':
    main()

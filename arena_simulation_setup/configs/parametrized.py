import os
import xml.etree.ElementTree as ET
from typing import Optional

import attrs

from arena_simulation_setup import Interface, ab_dir


def _get_attrib(
    element: ET.Element,
    attribute: str,
    default: Optional[str] = None
) -> str:
    val = element.get(attribute)
    if val is not None:
        return str(val)

    sub_elem = element.find(attribute)
    if sub_elem is not None:
        return str(sub_elem.text)

    if default is not None:
        return default

    raise ValueError(f"attribute {attribute} not found in {element}")


@attrs.define()
class ParametrizedConfig:
    @attrs.define()
    class ObstacleConfig:
        min: int
        max: int
        type: str
        model: str

    STATIC: list[ObstacleConfig]
    INTERACTIVE: list[ObstacleConfig]
    DYNAMIC: list[ObstacleConfig]


class Parametrized(Interface(os.path.join(ab_dir, 'configs', 'parametrized'))):
    def load(self) -> ParametrizedConfig:
        tree = ET.parse(self.path)
        root = tree.getroot()

        assert isinstance(
            root, ET.Element) and root.tag == "random", "not a random.xml desc"

        def xml_to_config(config) -> ParametrizedConfig.ObstacleConfig:
            return ParametrizedConfig.ObstacleConfig(
                min=int(_get_attrib(config, "min")),
                max=int(_get_attrib(config, "max")),
                type=_get_attrib(config, "type", ""),
                model=_get_attrib(config, "model")
            )

        return ParametrizedConfig(
            STATIC=list(map(xml_to_config, root.findall("./static/obstacle") or [])),
            INTERACTIVE=list(map(xml_to_config, root.findall("./static/interactive") or [])),
            DYNAMIC=list(map(xml_to_config, root.findall("./static/dynamic") or [])),
        )

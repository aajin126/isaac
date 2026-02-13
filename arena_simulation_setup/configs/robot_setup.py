import os
import typing

import yaml

from arena_simulation_setup import Interface, ab_dir


class RobotSetup(Interface(os.path.join(ab_dir, 'configs', 'robot_setup'))):

    def load(self) -> typing.Iterable[dict]:
        with open(self.path, 'r') as f:
            configuration = yaml.safe_load(f)

        result: list[dict] = []

        for entry_ in configuration:
            entry: dict = dict(entry_)
            amount = int(entry.pop('amount', 1))
            for _ in range(amount):
                result.append(entry)

        return result

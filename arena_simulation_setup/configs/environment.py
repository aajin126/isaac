import os

import yaml

from arena_simulation_setup import Interface, ass_dir


class Environment(Interface(os.path.join(ass_dir, 'configs', 'environment'))):
    def load(self):
        with open(self.path, 'r') as f:
            return yaml.safe_load(f)

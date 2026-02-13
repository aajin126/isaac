import os

from . import Model, ModelType, _ModelLoader

# raise RuntimeError('YAML models are not supported anymore')


@_ModelLoader.model(ModelType.YAML)
class ModelLoader_YAML(_ModelLoader):

    @classmethod
    def load(cls, model_dir, model, loader_args):

        model_path = os.path.join(model_dir, model, "yaml", f"{model}.yaml")

        try:
            with open(model_path) as f:
                model_desc = f.read()
        except FileNotFoundError:
            return None

        model_obj = Model(
            type=ModelType.YAML,
            name=model,
            description=model_desc,
            path=model_path
        )
        return model_obj

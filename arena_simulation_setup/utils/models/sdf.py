import os

from . import ITF_ModelLoader, Model, ModelType, _ModelLoader


@_ModelLoader.model(ModelType.SDF)
class ModelLoader_SDF(ITF_ModelLoader):

    @classmethod
    def load(cls, model_dir, model, loader_args):
        model_path = os.path.join(model_dir, model, "sdf", f"{model}.sdf")
        try:
            with open(model_path) as f:
                return Model(
                    type=ModelType.SDF,
                    name=model,
                    description=f.read(),
                    path=model_path
                )
        except FileNotFoundError:
            pass
        return None

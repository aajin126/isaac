import os
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from collections.abc import Collection

import arena_bringup

from . import ITF_ModelLoader, Model, ModelType, _ModelLoader


def process_dae(dae_file, package_dir):
    """
    Load a .dae file, update its <init_from> elements by replacing any leading
    '../' with the package_dir, then write to a temporary file and return its path.
    """
    import collada
    file = collada.Collada(dae_file)
    tree = file.xmlnode
    root = tree.getroot()
    for init_elem in root.iterfind('.//init_from'):
        print(init_elem)
        if init_elem.text:
            text = init_elem.text.strip()
            if text.startswith("../"):
                # Remove all leading "../" segments
                rel_path = text
                while rel_path.startswith("../"):
                    rel_path = rel_path[3:]
                # Create a new absolute path using the package directory
                new_text = os.path.join(package_dir, rel_path)
                init_elem.text = new_text

    with tempfile.NamedTemporaryFile(mode="wb", suffix=".dae", delete=False) as tmp_file:
        # Write the XML tree to the temporary file.
        tree.write(tmp_file, pretty_print=True, xml_declaration=True, encoding="UTF-8")
        temp_filename = tmp_file.name
    print(temp_filename)
    return temp_filename

    # Write the updated .dae file to a temporary file


def process_obj(obj_file, package_dir):
    """
    Read an .obj file as text and update any .png file references.
    For any found relative .png path (e.g. starting with "../"), remove the
    relative segments and prepend the package_dir. The modified file is saved
    to a temporary file whose path is returned.
    """
    try:
        with open(obj_file, 'r', encoding='utf-8') as f:
            content = f.read()
    except Exception as e:
        print(f"Error reading {obj_file}: {e}")
        return obj_file  # fallback: return original file if error occurs

    # Regex to match .png filenames (non-space characters ending in .png)
    png_pattern = re.compile(r'(?P<path>\S+\.png)')
    mtl_patter = re.compile(r'(?P<path>\S+\.mtl)')

    def replace_png(match):
        path = match.group("path")
        # If already absolute, do nothing.
        if os.path.isabs(path):
            return path
        # Remove any leading '../' segments
        while path.startswith("../"):
            path = path[3:]
        # Return the absolute path by joining with the package directory
        return os.path.join(package_dir, path)

    new_content = png_pattern.sub(replace_png, content)

    # Write the updated .obj file to a temporary file
    with tempfile.NamedTemporaryFile(delete=False, suffix='.obj', mode='w', encoding='utf-8') as temp_file:
        temp_file.write(new_content)
        temp_filename = temp_file.name
    print(temp_filename)
    return temp_filename


@_ModelLoader.model(ModelType.USD)
class ModelLoader_USD(ITF_ModelLoader):
    @classmethod
    def load(cls, model_dir, model, loader_args):
        model_path = os.path.join(model_dir, model, "usd", f"{model}.usd")
        try:
            with open(model_path, 'rb') as f:
                return Model(
                    type=ModelType.USD,
                    name=model,
                    description="",  # TODO add bytes compat
                    path=model_path
                )
        except FileNotFoundError:
            pass
        return None

    @classmethod
    def convertable(cls) -> Collection[ModelType]:
        return (ModelType.SDF,)

    @classmethod
    def convert(cls, model_dir: str, model: Model, loader_args) -> Model | None:
        if model.type == ModelType.SDF:
            try:
                # print(model_dir)
                sdf_model_path = os.path.join(model_dir, model.name, "sdf", f"{model.name}.sdf")
                materials_path = os.path.join(model_dir, model.name, 'sdf', 'materials')
                tree = ET.parse(sdf_model_path)
                root = tree.getroot()
                # First pass: resolve package:// URIs
                model_uri_pattern = re.compile(r'^model://([^/]+)(.*)$')
                package_uri_pattern = re.compile(r'^package://([^/]+)(.*)$')
                for uri_elem in root.iter():
                    if uri_elem.text:
                        text = uri_elem.text.strip()
                        match = model_uri_pattern.match(text)
                        if match:
                            package_name = match.group(1)
                            remaining_path = match.group(2)
                            # Get the absolute path for the package share directory.
                            # Replace the package URI with the resolved directory plus remaining path.
                            new_uri = model_dir + '/' + model.name + remaining_path
                            print(new_uri)
                            # uri_elem.text = new_uri
                            if (new_uri.endswith('.dae') or new_uri.endswith('.DAE')) and os.path.exists(new_uri):
                                new_dae_path = process_dae(new_uri, materials_path)
                                uri_elem.text = new_dae_path
                            elif new_uri.endswith('.obj') and os.path.exists(new_uri):
                                new_obj_path = process_obj(new_uri, materials_path)
                                uri_elem.text = new_obj_path
                        else:
                            match = package_uri_pattern.match(text)
                            if match:
                                package_name = match.group(1)
                                remaining_path = match.group(2)
                                # Get the absolute path for the package share directory.
                                # Replace the package URI with the resolved directory plus remaining path.
                                new_uri = model_dir + '/' + model.name + remaining_path
                                print(new_uri)
                                if (new_uri.endswith('.dae') or new_uri.endswith('.DAE')) and os.path.exists(new_uri):
                                    new_dae_path = process_dae(new_uri, materials_path)
                                    uri_elem.text = new_dae_path
                                elif new_uri.endswith('.obj') and os.path.exists(new_uri):
                                    new_obj_path = process_obj(new_uri, materials_path)
                                    uri_elem.text = new_obj_path
                model_path = os.path.join(model_dir, model.name, "usd", f"{model.name}.usd")
                os.makedirs(os.path.dirname(model_path), exist_ok=True)
                if os.path.islink(model_path) and not os.path.exists(model_path):  # broken symlink
                    os.unlink(model_path)

                ARENA_WS_DIR = arena_bringup.get_arena_ws_dir()

                env = os.environ.copy()
                env['ARENA_WS_DIR'] = ARENA_WS_DIR
                with tempfile.NamedTemporaryFile(delete=False, mode='w') as f:
                    tree.write(f, encoding='unicode')
                    f.flush()
                    temp_file_path = f.name
                    print("Temporary SDF file for converter:", temp_file_path)
                    subprocess.check_output(

                        [
                            f'{ARENA_WS_DIR}/src/arena/arena-rosnav/tools/sdf2usd',
                            f.name,
                            model_path
                        ],
                        env=env,
                        # shell=True,
                    )

                from pxr import Usd
                stage = Usd.Stage.Open(model_path)
                for prim in stage.Traverse():
                    if prim.GetTypeName() == "Xform":
                        first_xform_prim = prim
                        break
                else:
                    raise RuntimeError('no xform prim found')
                prim_path = first_xform_prim.GetPath()
                # print(prim_path)
                stage.SetDefaultPrim(first_xform_prim)
                root_layer = stage.GetRootLayer()
                root_layer.Save()
                return cls.load(model_dir, model.name, loader_args)

            except Exception:
                raise

        return None

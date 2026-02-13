import omni.isaac.nucleus

_assets_root_path: str = None


def get_assets_root_path_safe(fallback: str = '/', *, reload: bool = False) -> str:
    """
    Try to get nucleus root path, fall back to `fallback` if not determinable.
    """
    global _assets_root_path
    if not reload and _assets_root_path is not None:
        return _assets_root_path

    try:
        _assets_root_path = omni.isaac.nucleus.get_assets_root_path()
    except RuntimeError:
        _assets_root_path = fallback

    return _assets_root_path

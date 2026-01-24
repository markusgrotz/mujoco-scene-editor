from pathlib import Path
from typing import Optional

import trimesh


def convert_to_mujoco_mesh(
    src_path: Path,
    out_ext: str = ".obj",
    out_path: Optional[Path] = None,
) -> Path:
    """ """
    if out_ext not in (".stl", ".obj"):
        raise ValueError("out_ext must be '.stl' or '.obj'")

    if src_path.suffix.lower() == out_ext:
        return src_path

    mesh = trimesh.load_mesh(src_path)

    out_path = out_path or src_path.with_suffix(out_ext)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    mesh.export(out_path)

    return out_path

from typing import Iterable
from typing import List
from typing import Sequence
from typing import Optional

import datetime
from dataclasses import dataclass
from pathlib import Path

from cachier import cachier

ASSET_EXTS: Sequence[str] = (
    ".obj",
    ".stl",
    ".ply",
    ".glb",
    ".gltf",
    ".usd",
    ".usda",
    ".usdc",
)


@dataclass(frozen=True)
class ObjectModel:
    name: str
    path: Path


def _iter_asset_files(root: Path, exts: Iterable[str]) -> Iterable[Path]:
    exts_lc = tuple(e.lower() for e in exts)
    for p in root.rglob("*"):
        if p.is_file() and p.suffix.lower() in exts_lc:
            yield p


class Inventory:
    """
    Lightweight asset inventory backed by a directory scan.

    Use scan_assets_cached for caching, keep interface minimal.
    """

    @cachier(stale_after=datetime.timedelta(days=30))
    def list(self, root: Path) -> List[ObjectModel]:
        root = Path(root).expanduser().resolve()

        if not root.exists() or not root.is_dir():
            return []

        items = [
            ObjectModel(name=p.stem, path=p)
            for p in _iter_asset_files(root, ASSET_EXTS)
        ]
        items.sort(key=lambda m: m.name.lower())
        return items

    def list_names(self, root: Path) -> List[str]:
        return [m.name for m in self.list(root)]

    def find(self, root: Path, name: str) -> Optional[ObjectModel]:
        for m in self.list(root):
            if m.name == name:
                return m
        return None

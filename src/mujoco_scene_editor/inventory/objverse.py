from pathlib import Path
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Sequence


from dataclasses import dataclass
from functools import lru_cache

from cachier import cachier

import objaverse

ASSET_EXTS: Sequence[str] = (".glb", ".gltf", ".obj", ".ply")


@dataclass(frozen=True)
class ObjaverseItem:
    uid: str
    name: str
    path: Optional[Path] = None  # Local mesh path if downloaded/available


def _default_cache_root() -> Path:
    return Path.home() / ".objaverse"


@lru_cache()
def _find_local_asset(uid: str, exts: Sequence[str] = ASSET_EXTS) -> Optional[Path]:
    root = _default_cache_root()
    if not root.exists():
        return None
    # Typical structure: ~/.objaverse/hf-objaverse-v1/{uid}/.../*.{ext}
    uid_root = None
    for sub in root.iterdir():
        if sub.is_dir():
            cand = sub / uid
            if cand.exists() and cand.is_dir():
                uid_root = cand
                break
    if uid_root is None:
        # Fallback: brute-force search one level deep to avoid huge scans
        for sub in root.iterdir():
            cand = sub / uid
            if cand.exists() and cand.is_dir():
                uid_root = cand
                break
    if uid_root is None:
        return None
    exts_lc = tuple(e.lower() for e in exts)
    for p in uid_root.rglob("*"):
        if p.is_file() and p.suffix.lower() in exts_lc:
            return p
    return None


class ObjaverseInventory:
    """Inventory for Objaverse assets.

    - Lists UIDs by LVIS label.
    - Resolves local cached mesh paths if available.
    - Optionally downloads objects via objaverse when requested.
    """

    def list_by_label(
        self, label: str, limit: Optional[int] = None
    ) -> List[ObjaverseItem]:
        annotations = self._all_annotations()
        uids = annotations.get(label, []) or []
        if limit is not None:
            uids = uids[:limit]
        items: List[ObjaverseItem] = []
        for uid in uids:
            local = _find_local_asset(uid)
            name = local.stem if local else uid
            items.append(ObjaverseItem(uid=uid, name=name, path=local))
        return items

    @cachier(stale_after=None)
    def _all_annotations(self) -> Dict[str, List[str]]:
        return objaverse.load_lvis_annotations()

    def list_labels(self) -> List[str]:
        anns = self._all_annotations()
        return sorted(anns.keys())

    @cachier(stale_after=None)
    def list_by_labels(
        self,
        labels: Optional[Sequence[str]] = None,
        limit_per_label: Optional[int] = None,
    ) -> List[ObjaverseItem]:
        anns = self._all_annotations()
        labels = labels or list(anns.keys())
        results: List[ObjaverseItem] = []
        for label in labels:
            uids = list(anns.get(label, []))
            if limit_per_label is not None:
                uids = uids[:limit_per_label]
            for uid in uids:
                local = _find_local_asset(uid)
                name = local.stem if local else f"{label}"
                if any(it.name == name for it in results):
                    name = f"{name}_{uid[:6]}"
                results.append(ObjaverseItem(uid=uid, name=name, path=local))
        return sorted(results, key=lambda x: x.name)

    @cachier(stale_after=None)
    def list_all(self, limit: Optional[int] = None) -> List[ObjaverseItem]:
        """ """
        items: List[ObjaverseItem] = []
        for it in self.list_by_labels(labels=None, limit_per_label=None):
            items.append(it)
            if limit is not None and len(items) >= limit:
                break
        return items

    @cachier(stale_after=None)
    def list_local(self) -> List[ObjaverseItem]:
        root = _default_cache_root()
        if not root.exists():
            return []
        items: List[ObjaverseItem] = []
        for sub in root.iterdir():
            if not sub.is_dir():
                continue
            for uid_dir in sub.iterdir():
                if not uid_dir.is_dir():
                    continue
                uid = uid_dir.name
                p = _find_local_asset(uid)
                if p is None:
                    continue
                items.append(ObjaverseItem(uid=uid, name=p.stem, path=p))
        dedup: Dict[str, ObjaverseItem] = {it.uid: it for it in items}
        return sorted(dedup.values(), key=lambda x: x.name.lower())

    def download(self, uids: Iterable[str]) -> Dict[str, Path]:
        """Download objects via objaverse and return uid->local path mapping."""
        loaded = objaverse.load_objects(list(uids))
        return {uid: Path(p) for uid, p in loaded.items()}


SCALE_OVERRIDES: Dict[str, float] = {}


@cachier(stale_after=None)
def lookup_default_scale(_item: ObjaverseItem) -> float:
    return SCALE_OVERRIDES.get(_item.uid, 0.01)

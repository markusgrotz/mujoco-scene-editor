from typing import Dict
from typing import List

from copy import deepcopy
from dataclasses import replace
import logging

from robits.sim.blueprints import Blueprint
from robits.sim.blueprints import GripperBlueprint


logger = logging.getLogger(__name__)


def _is_path_or_descendant(path: str, root: str) -> bool:
    return path == root or path.startswith(f"{root}/")


class State:
    def __init__(self):
        self.blueprints: Dict[str, Blueprint] = {}
        self.history_past: List[Dict[str, Blueprint]] = []
        self.history_future: List[Dict[str, Blueprint]] = []
        self._seq = 0

    def sync_seq_from_blueprints(self) -> None:
        max_suffix = -1
        has_numeric_suffix = False
        for bp_name in self.blueprints.keys():
            leaf = bp_name.rsplit("/", maxsplit=1)[-1]
            if "_" not in leaf:
                continue
            suffix = leaf.rsplit("_", maxsplit=1)[-1]
            if suffix.isdigit():
                has_numeric_suffix = True
                max_suffix = max(max_suffix, int(suffix))
        if has_numeric_suffix:
            self._seq = max_suffix + 1
        else:
            self._seq = len(self.blueprints)

    def create_snapshot(self) -> Dict[str, Blueprint]:
        return deepcopy(self.blueprints)

    def push_state_to_history(self) -> None:
        self.history_past.append(self.create_snapshot())
        self.history_future.clear()

    def add(self, bp: Blueprint) -> None:
        self.push_state_to_history()
        self.blueprints[bp.path] = bp
        self._seq += 1

    def remove(self, bp_name: str) -> None:
        if bp_name not in self.blueprints:
            logger.error("Blueprint not found for removal: %s", bp_name)
            return
        self.push_state_to_history()
        for n in list(sorted(self.blueprints.keys())):
            if _is_path_or_descendant(n, bp_name):
                self.blueprints.pop(n)

    def update(self, bp_name: str, **kwargs) -> None:
        old = self.blueprints.get(bp_name, None)
        if old is None:
            logger.error("Blueprint %s not found for update: %s", bp_name, kwargs)
            return
        if isinstance(old, GripperBlueprint):
            logger.info("Not implemented yet")
            return
        self.push_state_to_history()
        self.blueprints[bp_name] = replace(old, **kwargs)

    def undo(self) -> bool:
        if not self.history_past:
            return False
        prev_state = self.history_past.pop()
        self.history_future.append(self.create_snapshot())
        self.blueprints = prev_state
        return True

    def redo(self) -> bool:
        if not self.history_future:
            return False
        future_state = self.history_future.pop()
        self.history_past.append(self.create_snapshot())
        self.blueprints = future_state
        return True

    def reset(self):
        self.blueprints.clear()
        self.history_past.clear()
        self.history_future.clear()
        self._seq = 0

    @property
    def element_seq(self) -> str:
        return f"{self._seq:04d}"

from typing import Sequence
from typing import Tuple
from typing import List

import logging
from pathlib import Path
from dataclasses import replace

from robits.core.config_manager import config_manager

from robits.sim.blueprints import Blueprint
from robits.sim.blueprints import GeomBlueprint
from robits.sim.blueprints import MeshBlueprint
from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import BlueprintGroup
from robits.sim.blueprints import Pose

from mujoco_scene_editor.state import State
from mujoco_scene_editor.utils import viser_utils
from mujoco_scene_editor.utils.blueprint_adapter import BlueprintAdapter

logger = logging.getLogger(__name__)


class SceneEditorController:
    def __init__(self, renderer) -> None:
        self.renderer = renderer
        self.state = State()
        self.is_running = True

    def load_blueprints(self, blueprints: List[Blueprint]):
        self.state.blueprints = {bp.path: bp for bp in blueprints}
        self.state.history_past.clear()
        self.state.history_future.clear()
        self.renderer.render_from_state(blueprints)

    def shutdown(self) -> None:
        self.is_running = False

    def create_group(self, parent_name: str, name: str) -> None:
        bp_name = self.get_full_name(parent_name, f"{name}_{self.state.element_seq}")
        bp = BlueprintGroup(bp_name, Pose())
        self.state.add(bp)
        self.renderer.add(bp)

        self.update_history_btn_visibility()

    def get_full_name(self, parent_name: str, name: str) -> str:
        return f"{parent_name}/{name}"

    def create_box(
        self, parent_name: str, dims: Sequence[float], rgba: Sequence[float]
    ) -> None:
        bp_name = self.get_full_name(parent_name, f"box_{self.state.element_seq}")
        half_size = [dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0]
        bp = GeomBlueprint(
            path=bp_name,
            geom_type="box",
            pose=Pose(),
            size=half_size,
            rgba=rgba,
            is_static=False,
        )
        self.state.add(bp)
        self.renderer.add(bp)

        self.update_history_btn_visibility()

    def create_cylinder(
        self, parent_name: str, radius: float, half_height: float, rgba: Sequence[float]
    ) -> None:
        bp_name = self.get_full_name(parent_name, f"cylinder_{self.state.element_seq}")
        bp = GeomBlueprint(
            path=bp_name,
            geom_type="cylinder",
            pose=Pose(),
            size=[radius, half_height],
            rgba=rgba,
            is_static=False,
        )
        self.state.add(bp)
        self.renderer.add(bp)

        self.update_history_btn_visibility()

    def create_sphere(
        self, parent_name: str, radius: float, rgba: Sequence[float]
    ) -> None:
        bp_name = self.get_full_name(parent_name, f"sphere_{self.state.element_seq}")
        bp = GeomBlueprint(
            path=bp_name,
            geom_type="sphere",
            pose=Pose(),
            size=[radius],
            rgba=rgba,
            is_static=False,
        )
        self.state.add(bp)
        self.renderer.add(bp)

        self.update_history_btn_visibility()

    def create_mesh(
        self, parent_name: str, mesh_path: Path, scale: float = 1.0
    ) -> None:
        bp_name = self.get_full_name(
            parent_name, f"asset_{mesh_path.name}_{self.state.element_seq}"
        )
        bp = MeshBlueprint(
            path=bp_name,
            mesh_path=str(mesh_path),
            pose=Pose(),
            is_static=False,
            scale=scale,
        )
        self.state.add(bp)
        self.renderer.add(bp)

        self.update_history_btn_visibility()

    def update_history_btn_visibility(self):
        self.renderer.layout.btn_undo.disabled = not bool(self.state.history_past)
        self.renderer.layout.btn_redo.disabled = not bool(self.state.history_future)

    def undo(self) -> None:
        has_changed = self.state.undo()
        if not has_changed:
            return
        self.renderer.render_from_state(list(self.state.blueprints.values()))
        self.update_history_btn_visibility()

    def redo(self) -> None:
        has_changed = self.state.redo()
        if not has_changed:
            return
        self.renderer.render_from_state(list(self.state.blueprints.values()))
        self.update_history_btn_visibility()

    def reset(self) -> None:
        self.state.reset()
        self.renderer.reset()
        self.update_history_btn_visibility()

    def select(self, name: str) -> None:
        self.renderer.on_select(name)

        if name not in self.state.blueprints:
            return

        bp = self.state.blueprints[name]

        if isinstance(bp, GeomBlueprint):
            self.renderer.layout.prop_element_mass.disabled = False
            self.renderer.layout.prop_element_mass.value = bp.mass


    def remove(self, name: str) -> None:
        self.state.remove(name)
        self.renderer.remove(name)
        self.update_history_btn_visibility()

    def export_scene(self, out_path: Path) -> None:
        # Sync robot joints from renderer without mutating history/state.
        all_joint_positions = self.renderer.get_joint_positions()

        bps_to_export: List[Blueprint] = []
        for name, bp in self.state.blueprints.items():
            if isinstance(bp, (RobotBlueprint, GripperBlueprint)):
                joint_positions = list(all_joint_positions.get(name, []))  # TODO M
                bp = replace(bp, default_joint_positions=joint_positions)
            bps_to_export.append(bp)

        out_path = out_path.with_suffix(".xml")
        blueprint_path = out_path.with_suffix(".json")
        out_path.parent.mkdir(parents=True, exist_ok=True)

        logger.info(
            "Exporting blueprints to %s and %s in folder %s",
            blueprint_path.name,
            out_path.name,
            out_path.parent,
        )

        # extract
        import json
        from robits.core.utils import MiscJSONEncoder

        blueprint_data = json.dumps(
            {"blueprints": [b for b in bps_to_export]},
            cls=MiscJSONEncoder,
            indent=3,
        )
        with open(blueprint_path, "w", encoding="utf-8") as f:
            f.write(blueprint_data)

        from robits.sim.converters.mujoco_exporter import MujocoXMLExporter

        MujocoXMLExporter().export_scene(out_path, bps_to_export)

    def update_pose(
        self,
        name: str,
        position: Tuple[float, float, float],
        wxyz: Tuple[float, float, float, float],
    ) -> None:
        new_pose = Pose().with_position(position).with_quat_wxyz(wxyz)
        self.state.update(name, pose=new_pose)
        self.renderer.update_pose(name, position, wxyz)
        self.update_history_btn_visibility()

    def create_camera(self, camera_name: str) -> None:
        adapter = BlueprintAdapter(camera_name)
        adapter.set_seq(self.state.element_seq)
        bp = adapter.to_camera_blueprint()

        self.state.add(bp)
        self.renderer.add(bp)

        self.update_history_btn_visibility()

    def create_robot(self, robot_config_name: str) -> None:
        config_dict = config_manager.load_dict(robot_config_name)
        robot_name = config_dict.get("robot_name", "robot")

        if left_config_name := config_dict.get("left_robot", None):
            self._add_robot_from_config(left_config_name, robot_name)

        if right_config_name := config_dict.get("right_robot", None):
            self._add_robot_from_config(right_config_name, robot_name)

        if "description_name" in config_dict:
            self._add_robot_from_config(robot_config_name, robot_name)

        self.update_history_btn_visibility()

    def _add_robot_from_config(self, config_name, robot_name: str) -> None:
        adapter = BlueprintAdapter(config_name)
        adapter.set_seq(self.state.element_seq)

        bp, gripper_bp = adapter.to_robot_bp(robot_name)

        if gripper_bp:
            self.state.add(gripper_bp)
            self.renderer.add(gripper_bp)

        self.state.add(bp)
        self.renderer.add_robot(bp, gripper_bp)

    def update_element(self, name: str, color, opacity, **kwargs) -> None:
        rgba = viser_utils.color_to_blueprint_rgba(color, opacity)
        self.state.update(name, rgba=rgba, **kwargs)
        self.renderer.update_element(name, color, opacity, **kwargs)
        self.update_history_btn_visibility()

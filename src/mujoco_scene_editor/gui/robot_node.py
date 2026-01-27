from typing import List
from typing import Tuple
from typing import Optional
from typing import Union

import numpy as np
import logging

from robot_descriptions.loaders.yourdfpy import load_robot_description

#  from viser import FrameHandle
from viser import TransformControlsHandle
from viser.extras import ViserUrdf

from mujoco_scene_editor.utils import viser_utils
from mujoco_scene_editor.utils import urdf_utils

logger = logging.getLogger(__name__)


class RobotNode:
    def __init__(
        self,
        layout,
        name: str,
        urdf_desc_name: str,
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
        create_eef_gizmo: bool = False,
    ) -> None:
        self.robot_base = layout.server.scene.add_frame(
            name, show_axes=False, position=position, wxyz=wxyz
        )

        if urdf_desc_name:
            urdf = load_robot_description(urdf_desc_name)
        else:
            logger.warning("Unable to find model. Using fallback model.")
            urdf = urdf_utils.fallback_urdf_model

        self.viser_urdf = ViserUrdf(
            layout.server, urdf_or_path=urdf, root_node_name=self.robot_base.name
        )
        with layout.server.gui.add_folder(f"Joint position control for {name}"):
            self.slider_handles, initial_config = (
                viser_utils.create_robot_control_sliders(layout.server, self.viser_urdf)
            )
        self.current_joint_positions = np.array(initial_config)

        for i, handle in enumerate(self.slider_handles):

            @handle.on_update
            def _(_e, idx=i, h=handle):
                self.current_joint_positions[idx] = h.value

        self.eef_gizmo: Optional[TransformControlsHandle] = None

        if create_eef_gizmo:
            gizmo_namespace = f"{self.robot_base.name}/eef_gizmo"
            self.eef_gizmo = layout.server.scene.add_transform_controls(
                gizmo_namespace, scale=0.2, visible=True
            )

        self.gripper_node: Optional["RobotNode"] = None

    @property
    def position(self) -> np.ndarray:
        return self.robot_base.position

    @position.setter
    def position(self, position: Union[Tuple[float, float, float], np.ndarray]) -> None:
        self.robot_base.position = position

    @property
    def wxyz(self) -> np.ndarray:
        return self.robot_base.wxyz

    @wxyz.setter
    def wxyz(self, wxyz: Union[Tuple[float, float, float, float], np.ndarray]) -> None:
        self.robot_base.wxyz = wxyz

    @property
    def name(self) -> Optional[str]:
        return self.robot_base.name

    def on_click(self, *args, **kwargs):
        return self.robot_base.on_click(*args, **kwargs)

    def remove(self) -> None:
        self.viser_urdf.remove()
        self.robot_base.remove()
        if self.eef_gizmo:
            self.eef_gizmo.remove()

    @property
    def visible(self) -> bool:
        return self.robot_base.visible

    @visible.setter
    def visible(self, visible: bool) -> None:
        self.robot_base.visible = visible
        if self.eef_gizmo:
            self.eef_gizmo.visible = visible

    def get_joint_positions(self) -> List[float]:
        return self.current_joint_positions.tolist()

    def attach_gripper_node(self, gripper_node: "RobotNode", attachment):
        self.gripper_node = gripper_node

        gripper_node.position = self.eef_gizmo.position
        gripper_node.wxyz = self.eef_gizmo.wxyz

        def move_gripper_cb(_e):
            gripper_node.position = self.eef_gizmo.position
            gripper_node.wxyz = self.eef_gizmo.wxyz

        self.eef_gizmo.on_update(move_gripper_cb)

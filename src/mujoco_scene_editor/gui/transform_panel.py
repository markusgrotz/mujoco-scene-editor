from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from mujoco_scene_editor.utils import viser_utils


class TransformPanel:
    def __init__(self, server) -> None:
        self._server = server
        self.position = server.gui.add_vector3(
            "Position (m)",
            initial_value=(0.0, 0.0, 0.0),
            step=0.01,
            disabled=False,
        )
        self.angles = server.gui.add_vector3(
            "Angles (deg)",
            initial_value=(0.0, 0.0, 0.0),
            step=1.0,
            disabled=False,
        )
        self.use_global_pose = server.gui.add_checkbox(
            "Use global pose", initial_value=False, disabled=True
        )
        self.btn_set_transform = server.gui.add_button("Set Transform")
        self.btn_reset = server.gui.add_button("Reset Transform")

    def on_select(self, node):
        self.use_global_pose.value = False
        self.position.value = node.position
        self.angles.value = viser_utils.wxyz_to_euler_deg(node.wxyz)

    def set_transform(
        self,
        position: Tuple[float, float, float],
        wxyz: Tuple[float, float, float, float],
    ) -> None:
        euler_deg = viser_utils.wxyz_to_euler_deg(wxyz)
        with self._server.atomic():  # TODO can we move this up?
            self.angles.value = euler_deg
            self.position.value = position

    def get_transform(
        self,
    ) -> Tuple[np.ndarray, np.ndarray]:
        position = self.position.value
        xyzw = R.from_euler("XYZ", self.angles.value, degrees=True).as_quat()
        return position, viser_utils.xyzw_to_wxyz(xyzw)

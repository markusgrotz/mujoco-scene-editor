from typing import Sequence
from typing import Tuple
from typing import List

import logging

import numpy as np
from scipy.spatial.transform import Rotation as R

import viser
from viser.extras import ViserUrdf

from robits.sim.blueprints import Blueprint


logger = logging.getLogger(__name__)


def parent_name(fullpath: str) -> str:
    return fullpath.rsplit("/", maxsplit=1)[0]


def base_name(fullpath: str) -> str:
    return fullpath.rsplit("/", maxsplit=1)[-1]


def wxyz_to_euler_deg(wxyz: Tuple[float, float, float, float]) -> np.ndarray:
    xyzw = wxyz_to_xyzw(wxyz)
    return R.from_quat(xyzw).as_euler("XYZ", degrees=True)


def wxyz_to_xyzw(q: Tuple[float, float, float, float]) -> np.ndarray:
    """Convert quaternion from (w, x, y, z) to (x, y, z, w)."""
    return np.concatenate((q[1:], q[:1]))


def xyzw_to_wxyz(q: Tuple[float, float, float, float]) -> np.ndarray:
    """Convert quaternion from (x, y, z, w) to (w, x, y, z)."""
    return np.concatenate((q[-1:], q[:-1]))


def create_robot_control_sliders(
    server: viser.ViserServer, viser_urdf: ViserUrdf
) -> Tuple[List[viser.GuiInputHandle[float]], List[float]]:
    """
    .. see:: https://viser.studio/main/examples/demos/urdf_visualizer/
    """
    slider_handles: list[viser.GuiInputHandle[float]] = []
    initial_config: list[float] = []
    for joint_name, (
        lower,
        upper,
    ) in viser_urdf.get_actuated_joint_limits().items():
        lower = lower if lower is not None else -np.pi
        upper = upper if upper is not None else np.pi
        initial_pos = 0.0 if lower < -0.1 and upper > 0.1 else (lower + upper) / 2.0
        slider = server.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=1e-3,
            initial_value=initial_pos,
        )
        slider.on_update(  # When sliders move, we update the URDF configuration.
            lambda _: viser_urdf.update_cfg(
                np.array([slider.value for slider in slider_handles])
            )
        )
        slider_handles.append(slider)
        initial_config.append(initial_pos)
    return slider_handles, initial_config


def pose_to_gui(
    bp: Blueprint,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """
    Extract a blueprint's pose into GUI-friendly (position, wxyz) tuples.
    Falls back to identity if the blueprint has no pose.
    """
    if pose := getattr(bp, "pose", None):
        position = tuple(v for v in pose.position)
        wxyz = tuple(v for v in pose.quaternion_wxyz)
    else:
        logger.warning("Invalid blueprint %s. Pose is None", bp)
        position = (0.0, 0.0, 0.0)
        wxyz = (1.0, 0.0, 0.0, 0.0)
    return position, wxyz


def color_from_blueprint(bp: Blueprint) -> Tuple[Tuple[int, int, int], float]:
    if rgba := getattr(bp, "rgba", None):
        color = (
            int(rgba[0] * 255),
            int(rgba[1] * 255),
            int(rgba[2] * 255),
        )
        opacity = rgba[3]
    else:
        logger.warning("Invalid blueprint %s. RGBA attribute is None", bp)
        color = (200, 200, 200)
        opacity = 1.0
    return color, opacity


def color_to_blueprint_rgba(
    color: Tuple[int, int, int], opacity: float = 1.0
) -> Sequence[float]:
    return [color[0] / 255.0, color[1] / 255.0, color[2] / 255.0, opacity]

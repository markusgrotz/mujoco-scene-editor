from typing import Tuple

import logging
from functools import lru_cache

import numpy as np

from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import Pose
from robits.sim.model_factory import SceneBuilder
from robits.sim import mjcf_utils

from robits.utils.transform_utils import transform_pose

from mujoco_scene_editor.utils import viser_utils

import mujoco
import mink

logger = logging.getLogger(__name__)


# IK parameters
SOLVER = "quadprog"
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 20


class SimpleIK:
    """
    Wrapper class around mink
    """

    def __init__(self, bp: RobotBlueprint, gripper_bp: GripperBlueprint):
        self.bp = bp
        self.gripper_bp = gripper_bp
        self.q = bp.default_joint_positions
        self.model, self.data = self._init_model()

        self.end_effector_task = mink.FrameTask(
            frame_name=self.attachment_site,
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        self.posture_task = mink.PostureTask(model=self.model, cost=1e-2)

    def _init_model(self):
        builder = SceneBuilder()
        builder.add_robot(self.bp, self.gripper_bp)
        builder.merge_all_keyframes_into_home()
        builder.add_mocap()
        model = mjcf_utils.reload_model_with_assets(builder.scene)

        data = mujoco.MjData(model)

        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)

        return model, data

    def reset_keyframe(self):
        model, data = self.model, self.data

        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)

    @property
    @lru_cache(maxsize=1)
    def attachment_site(self) -> str:
        if self.bp.attachment is None:
            logger.warning("No attachment specified")
            return "attachment_site"
        prefix = viser_utils.base_name(self.bp.path)
        return f"{prefix}/{self.bp.attachment.attachment_site}"

    def get_eef_pose(self):
        """
        TODO we should simply return the position of the attachment site and skip the mocap
        """

        model, data = self.model, self.data
        mink.move_mocap_to_frame(model, data, "target", self.attachment_site, "site")

        position, wxyz = data.mocap_pos[0].copy(), data.mocap_quat[0].copy()

        pose = self.bp.pose or Pose()
        transform = np.linalg.inv(pose.matrix)
        quat_xyzw = viser_utils.wxyz_to_xyzw(wxyz)

        new_position, new_xyzw = transform_pose(transform, position, quat_xyzw)

        return new_position, viser_utils.xyzw_to_wxyz(new_xyzw)

    def set_target(
        self,
        position: Tuple[float, float, float],
        wxyz: Tuple[float, float, float, float],
    ) -> np.ndarray:
        # if self.bp.attachment is None:
        #    self.bp.attachment = replace(self.bp, attachment=Attachment(blueprint_id="gripper_bp", wrist_name="hand"))

        model, data = self.model, self.data

        pose = self.bp.pose or Pose()
        transform = pose.matrix

        quat_xyzw = np.concatenate((wxyz[1:], wxyz[:1]))

        new_position, new_xyzw = transform_pose(transform, position, quat_xyzw)
        new_wxyz = np.concatenate((new_xyzw[-1:], new_xyzw[:-1]))

        # data.mocap_pos[0] = new_position
        # data.mocap_quat[0] = new_wxyz

        configuration = mink.Configuration(model, q=data.qpos)
        self.posture_task.set_target_from_configuration(configuration)

        # Update the end effector task target from the mocap body
        # T_wt = mink.SE3.from_mocap_name(model, data, "target")

        T_wt = mink.SE3(wxyz_xyz=np.concatenate([new_wxyz, new_position]))
        self.end_effector_task.set_target(T_wt)

        # Attempt to converge IK
        self.converge_ik(
            configuration,
            20,
            SOLVER,
            POS_THRESHOLD,
            ORI_THRESHOLD,
            MAX_ITERS,
        )

        # Step simulation
        mujoco.mj_step(model, data)

        return configuration.q

    def converge_ik(
        self, configuration, dt, solver, pos_threshold, ori_threshold, max_iters
    ):
        """
        Runs up to 'max_iters' of IK steps. Returns True if position and orientation
        are below thresholds, otherwise False.
        """
        tasks = [self.end_effector_task, self.posture_task]

        for _ in range(max_iters):
            vel = mink.solve_ik(configuration, tasks, dt, solver, 1e-3)
            configuration.integrate_inplace(vel, dt)

            # Only checking the first FrameTask here (end_effector_task).
            # If you want to check multiple tasks, sum or combine their errors.
            err = tasks[0].compute_error(configuration)
            pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
            ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold

            if pos_achieved and ori_achieved:
                return True
        return False

from typing import Optional
from typing import Tuple

import logging

import numpy as np
from robits.core.config_manager import config_manager

from robits.sim.blueprints import CameraBlueprint
from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import Attachment
from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import RobotDescriptionModel

from robits.sim.blueprints import Pose

from mujoco_scene_editor.utils import viser_utils

logger = logging.getLogger(__name__)


class BlueprintAdapter:
    def __init__(self, config_name: str):
        self.config_dict = config_manager.load_dict(config_name)
        self.seq = None

    def set_seq(self, seq: str) -> "BlueprintAdapter":
        self.seq = seq
        return self

    def to_camera_blueprint(self) -> CameraBlueprint:
        config_dict = self.config_dict
        camera_name = config_dict.get("camera_name", "camera")
        width = config_dict["width"]
        height = config_dict["height"]
        calibration_data = config_manager.load_dict(f"calibration_{camera_name}_camera")

        bp_name = f"/{camera_name}_{self.seq}"

        intrinsics = calibration_data["intrinsics"]
        extrinsics = calibration_data["extrinsics"]

        camera_pose = np.linalg.inv(extrinsics)

        return CameraBlueprint(bp_name, width, height, intrinsics, Pose(camera_pose))

    def to_robot_bp(
        self, robot_name: str
    ) -> Tuple[RobotBlueprint, Optional[GripperBlueprint]]:
        config_dict = self.config_dict
        if transform := config_dict.get("transform_robot_to_world", None):
            pose = Pose(np.asarray(transform).reshape((4, 4)))
        else:
            pose = Pose()

        description_name = config_dict["description_name"]
        variant_name = config_dict.get("variant_name", None)
        default_joint_positions = config_dict.get("default_joint_positions", None)

        if side_name := config_dict.get("side_name", None):
            bp_name = f"/{robot_name}_{side_name}_{self.seq}"
        else:
            bp_name = f"/{robot_name}_{self.seq}"

        attachment = None
        gripper_bp = None

        if "camera" in config_dict:
            logger.info("Skipping camera configuration: %s", config_dict["camera"])

        if "gripper" in config_dict:
            wrist_name = config_dict.get("wrist_name", "wrist")

            offset_pose = Pose()

            if quat := config_dict.get("wrist_quat", None):
                offset_pose = offset_pose.with_quat(np.fromstring(quat, sep=" "))
            if pos := config_dict.get("wrist_pos", None):
                offset_pose = offset_pose.with_position(np.fromstring(pos, sep=" "))

            gripper_config = config_manager.load_dict(config_dict["gripper"])
            gripper_description = RobotDescriptionModel(
                gripper_config["description_name"],
                gripper_config.get("variant_name", None),
            )
            gripper_namespace = f"{bp_name}/{gripper_config['gripper_name']}"
            gripper_bp = GripperBlueprint(gripper_namespace, model=gripper_description)
            attachment = Attachment(
                gripper_bp.path, wrist_name=wrist_name, attachment_offset=offset_pose
            )

        model_prefix = viser_utils.base_name(bp_name)

        model = RobotDescriptionModel(
            description_name=description_name,
            variant_name=variant_name,
            model_prefix_name=model_prefix,
        )

        bp = RobotBlueprint(
            path=bp_name,
            model=model,
            pose=pose,
            default_joint_positions=default_joint_positions,
            attachment=attachment,
        )
        return bp, gripper_bp

    def camera_bp_from_config(self) -> CameraBlueprint:
        config_dict = self.config_dict

        camera_name = config_dict.get("camera_name", "camera")
        width = config_dict["width"]
        height = config_dict["height"]
        calibration_data = config_manager.load_dict(f"calibration_{camera_name}_camera")

        bp_name = f"/{camera_name}_{self.seq}"

        intrinsics = calibration_data["intrinsics"]
        extrinsics = calibration_data["extrinsics"]

        camera_pose = np.linalg.inv(extrinsics)

        return CameraBlueprint(bp_name, width, height, intrinsics, Pose(camera_pose))

"""
Mapping utilities between MuJoCo description package names and URDF packages.

TODO this is incomplete as some models are missing or have a gripper already attached.
"""

import logging

logger = logging.getLogger(__name__)


def mj_to_urdf_description_name(desc: str) -> str:
    mujoco_description_name_to_urdf = {
        "ur5e_mj_description": "ur5_description",
        "ur10e_mj_description": "ur10_description",
        "robotiq_2f85_mj_description": "robotiq_2f85_description",
        "iiwa14_mj_description": "iiwa14_description",
        "gen3_mj_description": "gen3_description",
    }

    if desc in mujoco_description_name_to_urdf:
        return mujoco_description_name_to_urdf[desc]

    logger.warning("Unable to find matching URDF package for %s", desc)

    return desc.replace("_mj", "")

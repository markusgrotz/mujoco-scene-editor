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
        "iiwa14_mj_description": "iiwa14_description",
        # "xarm7_mj_description": None,
        "robotiq_2f85_mj_description": "robotiq_2f85_description",
        "gen3_mj_description": "gen3_description",
        "ability_hand_mj_description": "ability_hand_description",
        # "aero_hand_open_mj_description": "aero_hand_open_description",
        "allegro_hand_mj_description": "allegro_hand_description",
        "ability_hand_mj_description_./hands/abh_right_large.xml": "ability_hand_description",
        "panda_mj_description": "panda_description",
        "panda_mj_description_panda_nohand.xml": "panda_description",
        # "panda_mj_description_panda_hand.xml": "",
    }

    if desc in mujoco_description_name_to_urdf:
        return mujoco_description_name_to_urdf[desc]

    logger.warning("Unable to find matching URDF package for %s", desc)

    return ""

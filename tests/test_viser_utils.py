import unittest
import numpy as np
import numpy.testing as npt

from robits.sim.blueprints import Blueprint
from robits.sim.blueprints import Pose
from robits.sim.blueprints import GeomBlueprint

from mujoco_scene_editor.utils import viser_utils


class DummyBlueprint(Blueprint):
    def __init__(self, pose: Pose):
        super().__init__("/dummy")
        self.pose = pose


class NoPose(Blueprint):
    pass


class TestPoseToGui(unittest.TestCase):
    def test_pose_to_gui_with_valid_pose(self):
        pos = (1.0, 2.0, 3.0)
        quat = (0.0, 0.0, 0.0, 1.0)
        pose = Pose().with_position(pos).with_quat(quat)
        bp = DummyBlueprint(pose)

        position, wxyz = viser_utils.pose_to_gui(bp)

        self.assertEqual(position, pos)
        self.assertEqual((1.0, 0.0, 0.0, 0.0), wxyz)

    def test_pose_to_gui_without_pose(self):
        bp = NoPose("/no_pose")
        position, wxyz = viser_utils.pose_to_gui(bp)

        self.assertEqual(position, (0.0, 0.0, 0.0))
        self.assertEqual(wxyz, (1.0, 0.0, 0.0, 0.0))

    def test_geom_blueprint(self):
        pose = Pose().with_quat_wxyz([1, 0, 0, 0])

        bp = GeomBlueprint("/", pose=pose)

        position, wxyz = viser_utils.pose_to_gui(bp)

        self.assertEqual(position, (0.0, 0.0, 0.0))
        self.assertEqual(wxyz, (1.0, 0.0, 0.0, 0.0))


class TestQuaternionConversions(unittest.TestCase):
    def test_wxyz_to_xyzw(self):
        wxyz = (1.0, 0.0, 0.0, 0.0)
        xyzw = viser_utils.wxyz_to_xyzw(wxyz)
        expected = np.array([0.0, 0.0, 0.0, 1.0])
        npt.assert_array_equal(xyzw, expected)

    def test_xyzw_to_wxyz(self):
        xyzw = (0.0, 0.0, 0.0, 1.0)
        wxyz = viser_utils.xyzw_to_wxyz(xyzw)
        expected = np.array([1.0, 0.0, 0.0, 0.0])
        npt.assert_array_equal(wxyz, expected)

    def test_roundtrip_conversions(self):
        wxyz = (0.5, 0.5, 0.5, 0.5)
        xyzw = (0.5, -0.5, 0.5, -0.5)

        npt.assert_array_equal(
            viser_utils.xyzw_to_wxyz(viser_utils.wxyz_to_xyzw(wxyz)),
            np.array(wxyz),
        )

        npt.assert_array_equal(
            viser_utils.wxyz_to_xyzw(viser_utils.xyzw_to_wxyz(xyzw)),
            np.array(xyzw),
        )


if __name__ == "__main__":
    unittest.main()

import unittest


from robits.sim.blueprints import BlueprintGroup
from mujoco_scene_editor.state import State


class TestState(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        self.state = State()

    def test_add(self):
        bp = BlueprintGroup("/foo")
        self.state.add(bp)

        self.assertDictEqual(self.state.blueprints, {"/foo": bp})

    def test_remove(self):
        self.state.add(BlueprintGroup("/foo"))
        self.state.add(BlueprintGroup("/foo/bar"))
        self.state.add(BlueprintGroup("/other"))

        self.state.remove("/other")
        self.assertDictEqual(
            self.state.blueprints,
            {"/foo": BlueprintGroup("/foo"), "/foo/bar": BlueprintGroup("/foo/bar")},
        )

        self.state.add(BlueprintGroup("/other"))
        self.state.remove("/foo")
        self.assertDictEqual(
            self.state.blueprints, {"/other": BlueprintGroup("/other")}
        )

    def test_remove_does_not_delete_prefix_sibling(self):
        self.state.add(BlueprintGroup("/foo"))
        self.state.add(BlueprintGroup("/foo/bar"))
        self.state.add(BlueprintGroup("/foobar"))

        self.state.remove("/foo")

        self.assertDictEqual(
            self.state.blueprints,
            {"/foobar": BlueprintGroup("/foobar")},
        )

    def test_undo_after_remove(self):
        self.state.add(BlueprintGroup("/foo"))
        self.state.add(BlueprintGroup("/foo/bar"))

        self.state.remove("/foo")
        self.state.undo()

        self.assertDictEqual(
            self.state.blueprints,
            {"/foo": BlueprintGroup("/foo"), "/foo/bar": BlueprintGroup("/foo/bar")},
        )

    def test_undo_redo(self):
        bp = BlueprintGroup("/foo")
        self.state.add(bp)

        self.state.undo()
        self.assertDictEqual(self.state.blueprints, {})

        self.state.redo()
        self.assertDictEqual(self.state.blueprints, {"/foo": bp})

    def test_element_seq_does_not_reuse_after_remove(self):
        self.state.add(BlueprintGroup("/box_0000"))
        self.state.add(BlueprintGroup("/box_0001"))

        self.state.remove("/box_0000")

        self.assertEqual(self.state.element_seq, "0002")

    def test_sync_seq_from_blueprints_with_numeric_suffix(self):
        self.state.blueprints = {
            "/group/box_0999": BlueprintGroup("/group/box_0999"),
            "/group/table": BlueprintGroup("/group/table"),
        }

        self.state.sync_seq_from_blueprints()

        self.assertEqual(self.state.element_seq, "1000")

    def test_sync_seq_from_blueprints_without_numeric_suffix(self):
        self.state.blueprints = {
            "/group/table": BlueprintGroup("/group/table"),
            "/group/lamp": BlueprintGroup("/group/lamp"),
        }

        self.state.sync_seq_from_blueprints()

        self.assertEqual(self.state.element_seq, "0002")


if __name__ == "__main__":
    unittest.main()

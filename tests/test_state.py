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


if __name__ == "__main__":
    unittest.main()

from typing import Tuple
from typing import Dict
from typing import Any
from typing import Optional
from typing import List

import logging

import math
from pathlib import Path

from functools import singledispatchmethod

import trimesh

from viser import SceneNodeHandle
from viser import SceneApi

from robits.sim.blueprints import GeomBlueprint
from robits.sim.blueprints import MeshBlueprint
from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import Blueprint
from robits.sim.blueprints import CameraBlueprint
from robits.sim.blueprints import BlueprintGroup

from mujoco_scene_editor.utils.simple_ik import SimpleIK


from mujoco_scene_editor.gui.robot_node import RobotNode
from mujoco_scene_editor.utils.mj_urdf_map import mj_to_urdf_description_name
from mujoco_scene_editor.utils import viser_utils
from mujoco_scene_editor.constants import NO_SELECTION


logger = logging.getLogger(__name__)


class ViserSceneRenderer:
    def __init__(self, layout) -> None:
        self.name_to_node: Dict[str, SceneNodeHandle] = {}  #
        self.layout = layout

    def render_from_state(self, blueprints: List[Blueprint]):
        """
        TODO can we batch this?
        """
        self.reset()
        blueprints.sort(key=lambda bp: bp.path)
        gripper_bps: Dict[str, GripperBlueprint] = {}
        for bp in blueprints:
            if isinstance(bp, RobotBlueprint) and bp.attachment:
                continue
            if isinstance(bp, GripperBlueprint):
                gripper_bps[bp.path] = bp
            self.add(bp)

        for bp in blueprints:
            if isinstance(bp, RobotBlueprint) and bp.attachment:
                self.add_robot(bp, gripper_bps.get(bp.attachment.gripper_path))

    def add(self, bp: Blueprint):
        logger.debug("Adding blueprint %s", bp)
        node = self._create_node(bp)
        self.register_node(node)
        return node

    def replace(self, bp: Blueprint) -> SceneNodeHandle:
        if node := self.name_to_node.pop(bp.path, None):
            node.remove()
        return self.add(bp)

    @singledispatchmethod
    def _create_node(self, bp: Blueprint) -> SceneNodeHandle:
        raise NotImplementedError(
            "Function is not implemented for Blueprint type %s", type(bp)
        )

    @_create_node.register
    def _create_group_node(self, bp: BlueprintGroup) -> SceneNodeHandle:
        position, wxyz = viser_utils.pose_to_gui(bp)
        return self.layout.server.scene.add_frame(
            bp.path,
            show_axes=False,
            position=position,
            wxyz=wxyz,
        )

    @_create_node.register
    def _create_camera_node(self, bp: CameraBlueprint) -> SceneNodeHandle:
        position, wxyz = viser_utils.pose_to_gui(bp)
        fx = bp.intrinsics[0][0]
        fov = 2.0 * math.atan2(bp.width / 2.0, fx)
        aspect_ratio = bp.width / bp.height
        from scipy.spatial.transform import Rotation as R

        xyzw = viser_utils.wxyz_to_xyzw(wxyz)
        rotated_xyzw = (
            R.from_quat(xyzw) * R.from_euler("XYZ", [0, 3.141592, 0.0])
        ).as_quat()
        rotated_wxyz = viser_utils.xyzw_to_wxyz(rotated_xyzw)
        node = self.layout.server.scene.add_camera_frustum(
            bp.path, fov=fov, aspect=aspect_ratio, position=position, wxyz=rotated_wxyz
        )
        return node

    @_create_node.register
    def _create_geom_node(self, bp: GeomBlueprint) -> SceneNodeHandle:
        color, opacity = viser_utils.color_from_blueprint(bp)
        position, wxyz = viser_utils.pose_to_gui(bp)
        api: SceneApi = self.layout.server.scene

        node_name = bp.path

        if bp.geom_type == "box":
            extends = (bp.size[0] * 2, bp.size[1] * 2, bp.size[2] * 2)
            node = api.add_box(
                node_name,
                dimensions=extends,
                color=color,
                opacity=opacity,
                position=position,
                wxyz=wxyz,
                flat_shading=True,
            )

        elif bp.geom_type == "plane":
            extends = (bp.size[0] * 2, bp.size[1] * 2, 0.001)
            node = api.add_box(
                node_name,
                dimensions=extends,
                color=color,
                opacity=opacity,
                position=position,
                wxyz=wxyz,
                flat_shading=True,
            )
        #    node = api.add_frame(element_name)
        #    grid = api.add_grid(
        #        f"{element_name}/grid",
        #        width=bp.size[0] * 2,
        #        height=bp.size[1] *2,
        #        #height_segments=bp.size[2],
        #        #cell_color=color,
        #        section_color=color,
        #        position=position,
        #        wxyz=wxyz
        #    )

        elif bp.geom_type == "cylinder":
            radius = bp.size[0]
            height = bp.size[1] * 2.0
            node = api.add_cylinder(
                node_name,
                radius,
                height,
                color=color,
                opacity=opacity,
                position=position,
                wxyz=wxyz,
            )

        elif bp.geom_type == "capsule":
            radius = bp.size[0]
            height = bp.size[1] * 2.0
            tri = trimesh.creation.capsule(radius=radius, height=height)
            tri.visual.face_colors = color[:3]
            node = api.add_mesh_trimesh(
                node_name,
                mesh=tri,
                position=position,
                wxyz=wxyz,
            )
        elif bp.geom_type == "ellipsoid":
            tri = trimesh.creation.icosphere(subdivisions=4, radius=1.0)
            tri.apply_scale(bp.size)
            tri.visual.face_colors = color[:3]
            node = api.add_mesh_trimesh(
                node_name,
                mesh=tri,
                position=position,
                wxyz=wxyz,
            )
        elif bp.geom_type == "sphere":
            radius = bp.size[0]
            node = api.add_icosphere(
                node_name,
                radius=radius,
                color=color,
                opacity=opacity,
                position=position,
                wxyz=wxyz,
            )
        else:
            logger.error("Unable to parse blueprint %s", bp)
            raise ValueError("Unsupported geom type. %s", bp.geom_type)

        return node

    @_create_node.register
    def _create_mesh_node(self, bp: MeshBlueprint) -> SceneNodeHandle:
        position, wxyz = viser_utils.pose_to_gui(bp)
        tri = trimesh.load_mesh(str(Path(bp.mesh_path).resolve()))
        scale = getattr(bp, "scale", None)
        if scale is not None:
            tri.apply_scale(scale)

        node = self.layout.server.scene.add_mesh_trimesh(
            bp.path, mesh=tri, wxyz=wxyz, position=position
        )
        return node

    @_create_node.register
    def _create_gripper_node(self, bp: GripperBlueprint):
        desc = bp.model.description_name
        variant_name = bp.model.variant_name
        position, wxyz = viser_utils.pose_to_gui(bp)

        if variant_name:
            logger.warning(
                "Variant %s not implemented. Appending to description name.",
                variant_name,
            )
            desc = f"{desc}_{variant_name}"
        urdf_desc_name = mj_to_urdf_description_name(desc)
        robot_node = RobotNode(
            self.layout, bp.path, urdf_desc_name, position, wxyz, create_eef_gizmo=False
        )

        if bp.default_joint_positions:
            for slider, q in zip(robot_node.slider_handles, bp.default_joint_positions):
                slider.value = q
        return robot_node

    @_create_node.register
    def _create_robot_node(self, bp: RobotBlueprint):
        if bp.attachment:
            logger.warning("Not implemented yet. Use add_robot instead")

        desc = bp.model.description_name
        variant_name = bp.model.variant_name
        position, wxyz = viser_utils.pose_to_gui(bp)
        if variant_name:
            logger.warning(
                "Variant %s not implemented. Appending to description name.",
                variant_name,
            )
            desc = f"{desc}_{variant_name}"
        urdf_desc_name = mj_to_urdf_description_name(desc)
        robot_node = RobotNode(
            self.layout, bp.path, urdf_desc_name, position, wxyz, create_eef_gizmo=True
        )

        if bp.default_joint_positions:
            for slider, q in zip(robot_node.slider_handles, bp.default_joint_positions):
                slider.value = q
        return robot_node

    def add_robot(
        self, bp: RobotBlueprint, gripper_bp: Optional[GripperBlueprint] = None
    ):
        desc = bp.model.description_name
        variant_name = bp.model.variant_name
        position, wxyz = viser_utils.pose_to_gui(bp)
        if variant_name:
            logger.warning(
                "Variant %s not implemented. Appending to description name.",
                variant_name,
            )
            desc = f"{desc}_{variant_name}"

        urdf_desc_name = mj_to_urdf_description_name(desc)
        has_gripper = bool(gripper_bp and bp.attachment)

        robot_node = RobotNode(
            self.layout,
            bp.path,
            urdf_desc_name,
            position,
            wxyz,
            create_eef_gizmo=has_gripper,
        )

        if has_gripper:
            solver = SimpleIK(bp, gripper_bp)
            eef_position, eef_wxyz = solver.get_eef_pose()
            robot_node.eef_gizmo.position = eef_position
            robot_node.eef_gizmo.wxyz = eef_wxyz

            @robot_node.eef_gizmo.on_update
            def _(_e):
                joint_positions = solver.set_target(
                    robot_node.eef_gizmo.position, robot_node.eef_gizmo.wxyz
                )
                joint_positions = joint_positions[: len(robot_node.slider_handles)]
                robot_node.current_joint_positions = joint_positions
                for slider, q in zip(robot_node.slider_handles, joint_positions):
                    slider.value = q

            _(None)

            if gripper_node := self.name_to_node.get(gripper_bp.path, None):
                robot_node.attach_gripper_node(gripper_node, bp.attachment)
            else:
                logger.error("Unable to find gripper node. %s", bp.attachment)
        self.register_node(robot_node)

        return robot_node

    def register_node(self, node: SceneNodeHandle):
        node_name = node.name
        self.name_to_node[node_name] = node

        self.update_elements_dropdown(node_name)

        @node.on_click
        def _(_evt) -> None:
            if self.layout.allow_mouse_select.value:
                self.layout.element_list.value = node.name

    def get_joint_positions(self) -> Dict[str, List[float]]:
        all_joint_positions = {}
        for name, node in self.name_to_node.items():
            if isinstance(node, RobotNode):
                all_joint_positions[name] = node.get_joint_positions()
        return all_joint_positions

    def update_elements_dropdown(self, selected_value: Optional[str] = None) -> None:
        previous_value = self.layout.element_list.value
        names = (NO_SELECTION,) + tuple(self.name_to_node.keys())
        self.layout.element_list.options = names
        if selected_value:
            self.layout.element_list.value = selected_value
        elif previous_value not in names:
            self.layout.element_list.value = names[0]

    def node_to_global_pose(self, node_name: str) -> Tuple[Any, Any]:
        import numpy as np
        from scipy.spatial.transform import Rotation as R

        global_pose = np.identity(4)

        prefix = []
        for n in node_name.split("/"):
            prefix.append(n)
            if not n:
                continue
            current_node_path = "/".join(prefix)
            current_node = self.name_to_node.get(current_node_path, None)
            if current_node is None:
                logger.warning("Unable to find node with path %s", current_node_path)
                continue
            position, wxyz = current_node.position, current_node.wxyz
            current_transform = np.identity(4)
            current_transform[:3, :3] = R.from_quat(
                viser_utils.wxyz_to_xyzw(wxyz)
            ).as_matrix()
            current_transform[:3, 3] = position
            global_pose = np.dot(global_pose, current_transform)

        position = global_pose[:3, 3]
        xyzw = R.from_matrix(global_pose[:3, :3]).as_quat()

        return position, viser_utils.xyzw_to_wxyz(xyzw)

    def update_pose(
        self,
        node_name: str,
        position: Tuple[float, float, float],
        wxyz: Tuple[float, float, float, float],
    ) -> None:
        node = self.name_to_node[node_name]

        # synchronize the node and the gizmo
        node.wxyz = wxyz
        node.position = position

        # also update the gizmo since we don't know who called this
        self.layout.gizmo.position = position
        self.layout.gizmo.wxyz = wxyz

        global_position, global_wxyz = self.node_to_global_pose(node_name)

        # TODO this is not the global position. We need to get the global position somehow
        self.layout.transform.set_transform(position, wxyz)

    def remove(self, node_name: str) -> None:
        for name in list(sorted(self.name_to_node.keys())):
            if name.startswith(node_name):
                node = self.name_to_node.pop(name)
                node.remove()

        if self.layout.gizmo:
            self.layout.gizmo.remove()
            self.layout.gizmo = None

        self.update_elements_dropdown()

    def reset(self) -> None:
        for _name, node in self.name_to_node.items():
            node.remove()
        self.name_to_node.clear()

        self.update_elements_dropdown()

    def on_select(self, node_name: str) -> None:
        """
        TODO adjust the scale of the gizmo to the object size.
        """
        # Remove previous gizmo if any

        if node_name not in self.name_to_node:
            logger.warning("Unable to select node with name %s", node_name)
            return

        self.configure_properties_panel(node_name)
        node = self.name_to_node[node_name]
        self.layout.on_select(node)

    def configure_properties_panel(self, node_name: str):
        self.layout.disable_all_properties_gui_elements()
        node = self.name_to_node[node_name]
        if hasattr(node, "color") and hasattr(node, "opacity"):
            self.layout.prop_color.value = node.color
            self.layout.prop_opacity.value = node.opacity
            self.layout.prop_color.disabled = False
            self.layout.prop_opacity.disabled = False
        if hasattr(node, "dimensions"):
            self.layout.prop_box_dims.value = node.dimensions
            self.layout.prop_box_dims.disabled = False
        elif hasattr(node, "height"):
            self.layout.prop_cyl_radius.value = node.radius
            self.layout.prop_cyl_height.value = node.height
            self.layout.prop_cyl_radius.disabled = False
            self.layout.prop_cyl_height.disabled = False
        elif hasattr(node, "radius"):
            self.layout.prop_sphere_radius.value = node.radius
            self.layout.prop_sphere_radius.disabled = False

    def update_element(self, node_name: str, color, opacity, **kwargs):
        if node_name not in self.name_to_node:
            return

        node = self.name_to_node[node_name]

        node.color = color
        node.opacity = opacity

        if "size" in kwargs:
            kwargs["dimensions"] = kwargs["size"]
            kwargs["radius"] = kwargs["size"][0]
            kwargs["height"] = kwargs["size"][1]

        for k, v in kwargs.items():
            if hasattr(node, k):
                setattr(node, k, v)
            else:
                logger.warning("Attribute not found %s", k)

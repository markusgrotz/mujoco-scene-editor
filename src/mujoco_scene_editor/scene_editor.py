import sys
import logging
from pathlib import Path
import subprocess

from mujoco_scene_editor.layout import SceneEditorLayout
from mujoco_scene_editor.inventory.local_assets import ObjectModel
from mujoco_scene_editor.inventory.local_assets import Inventory
from mujoco_scene_editor.inventory.objverse import ObjaverseInventory
from mujoco_scene_editor.inventory.objverse import ObjaverseItem
from mujoco_scene_editor.inventory.objverse import lookup_default_scale
from mujoco_scene_editor.inventory.objverse import SCALE_OVERRIDES

from mujoco_scene_editor.utils.viser_utils import color_to_blueprint_rgba
from mujoco_scene_editor.utils.mesh_conversion import convert_to_mujoco_mesh

from mujoco_scene_editor.constants import NO_SELECTION

from viser import GuiEvent

logger = logging.getLogger(__name__)


class SceneEditor:
    def __init__(self, controller, layout: SceneEditorLayout) -> None:
        self.controller = controller
        self.layout = layout

        self.inventory = Inventory()
        self.obj_inventory = ObjaverseInventory()

        self._load_inventory()
        self._register_cbs()

    def _load_inventory(self):
        items = self.inventory.list(
            root=Path(self.layout.assets_dir.value.strip()).expanduser()
        )
        self.layout.asset_items = {m.name: m for m in items}
        self.layout.update_assets_dropdown()

        self.layout.objaverse_labels = tuple(self.obj_inventory.list_labels())
        self.layout.update_objaverse_label_dropdown()

        obj_items = self.obj_inventory.list_by_labels(labels=None, limit_per_label=50)
        self.layout.objaverse_items = {m.name: m for m in obj_items}
        self.layout.update_objaverse_dropdown()

    def _register_cbs(self):
        self.layout.btn_clear.on_click(self.reset_scene)
        self.layout.btn_quit.on_click(self.quit_server)

        self.layout.btn_undo.on_click(self.undo)
        self.layout.btn_redo.on_click(self.redo)

        self.layout.btn_create_box.on_click(self.create_box)
        self.layout.btn_create_cylinder.on_click(self.create_cylinder)
        self.layout.btn_create_sphere.on_click(self.create_sphere)
        self.layout.btn_add_camera.on_click(self.add_camera)
        self.layout.btn_add_robot.on_click(self.add_robot)

        self.layout.btn_delete_element.on_click(self.delete_element)
        self.layout.element_list.on_update(self.on_select)
        self.layout.transform.btn_reset.on_click(self.reset_selected_transform)
        self.layout.transform.btn_set_transform.on_click(self.set_selected_transform)
        self.layout.btn_scan_assets.on_click(self.scan_assets)
        self.layout.btn_add_asset.on_click(self.add_asset)
        self.layout.btn_add_objaverse.on_click(self.add_objaverse_object)
        self.layout.objaverse_scale.on_update(self.on_objaverse_scale_change)
        self.layout.objaverse_category_list.on_update(self.on_objaverse_category_change)
        self.layout.objaverse_list.on_update(self.on_objaverse_item_change)

        self.layout.btn_export_mj.on_click(self.export_mujoco)
        self.layout.btn_launch_mj.on_click(self.launch_mujoco_viewer)

        self.layout.btn_update_element.on_click(self.update_element)
        self.layout.enable_gizmo_checkbox.on_update(self.toggle_gizmo_visibility)
        self.layout.btn_create_group.on_click(self.create_group)

    def toggle_gizmo_visibility(self, _evt: GuiEvent) -> None:
        self.layout.gizmo.visible = self.layout.enable_gizmo_checkbox.value

    def on_gizmo_drag_end(self, _evt: GuiEvent) -> None:
        gizmo = self.layout.gizmo
        sel = self.layout.element_list.value
        self.controller.update_pose(sel, gizmo.position, gizmo.wxyz)

    @property
    def url(self) -> str:
        server = self.layout.server
        # FIXME we have to access the private variables since it is possible to override the port with environment variables.
        port = server._websock_server._port
        return f"http://localhost:{port}"
        # return server.request_share_url()

    def show(self) -> None:
        logger.info("Open the Scene Editor UI in your browser: %s", self.url)

    def reset_scene(self, _evt: GuiEvent) -> None:
        self.layout.reset()
        self.controller.reset()

    @property
    def is_running(self) -> bool:
        return self.controller.is_running

    def quit_server(self, _evt: GuiEvent) -> None:
        self.layout.server.stop()
        self.controller.shutdown()

    def undo(self, _evt: GuiEvent) -> None:
        self.controller.undo()

    def redo(self, _evt: GuiEvent) -> None:
        self.controller.redo()

    def get_selected_parent(self) -> str:
        sel = self.layout.element_list.value
        if sel == NO_SELECTION:
            return ""
        return sel

    def create_group(self, _evt: GuiEvent) -> None:
        name = self.layout.txt_group_name.value
        parent_name = self.get_selected_parent()
        self.controller.create_group(parent_name, name)

    def create_box(self, _evt: GuiEvent) -> None:
        parent_name = self.get_selected_parent()
        dims = self.layout.box_dims.value
        color = self.layout.box_color.value  # RGB 0–255
        opacity = self.layout.box_opacity.value
        rgba = color_to_blueprint_rgba(color, opacity)
        self.controller.create_box(parent_name, dims, rgba)

    def create_cylinder(self, _evt: GuiEvent) -> None:
        parent_name = self.get_selected_parent()
        radius = self.layout.cyl_radius.value
        height = self.layout.cyl_height.value
        half_height = 0.5 * height
        color = self.layout.cyl_color.value  # RGB 0–255
        opacity = self.layout.cyl_opacity.value
        rgba = color_to_blueprint_rgba(color, opacity)
        self.controller.create_cylinder(parent_name, radius, half_height, rgba)

    def create_sphere(self, _evt: GuiEvent) -> None:
        parent_name = self.get_selected_parent()
        radius = self.layout.sphere_radius.value
        color = self.layout.sphere_color.value
        opacity = self.layout.sphere_opacity.value
        rgba = color_to_blueprint_rgba(color, opacity)
        self.controller.create_sphere(parent_name, radius, rgba)

    def delete_element(self, _evt: GuiEvent) -> None:
        sel = self.layout.element_list.value
        self.controller.remove(sel)

    def on_select(self, _evt: GuiEvent) -> None:
        sel = self.layout.element_list.value

        self.controller.select(sel)

        if self.layout.gizmo:
            self.layout.gizmo.on_drag_end(self.on_gizmo_drag_end)

    def reset_selected_transform(self, _evt: GuiEvent) -> None:
        sel = self.layout.element_list.value
        position = (0.0, 0.0, 0.0)
        wxyz = (1.0, 0.0, 0.0, 0.0)
        self.controller.update_pose(sel, position, wxyz)

    def set_selected_transform(self, _evt: GuiEvent) -> None:
        sel = self.layout.element_list.value
        position, wxyz = self.layout.transform.get_transform()
        self.controller.update_pose(sel, position, wxyz)

    def scan_assets(self, _evt: GuiEvent) -> None:
        root = self.layout.assets_dir.value.strip() or "~"
        items = self.inventory.list(
            root=Path(root).expanduser(), cachier__overwrite_cache=True
        )
        self.layout.asset_items = {m.name: m for m in items}
        self.layout.update_assets_dropdown()

    def add_asset(self, _evt: GuiEvent) -> None:
        sel = self.layout.assets_list.value
        if sel not in self.layout.asset_items:
            return
        model: ObjectModel = self.layout.asset_items[sel]
        parent_name = self.get_selected_parent()
        self.controller.create_mesh(parent_name, model.path.resolve())

    def add_objaverse_object(self, event: GuiEvent) -> None:
        sel = self.layout.objaverse_list.value
        if sel not in self.layout.objaverse_items:
            return

        item: ObjaverseItem = self.layout.objaverse_items[sel]
        mesh_path = item.path
        if mesh_path is None:
            self.layout.btn_add_objaverse.disabled = True
            client = event.client
            notification = client.add_notification(
                title="Download in progress",
                body=f"Downloading objaverse item {item.name}",
                loading=True,
            )
            local_map = self.obj_inventory.download([item.uid])
            notification.remove()
            self.layout.btn_add_objaverse.disabled = False

            mesh_path = local_map.get(item.uid)
            if mesh_path is None:
                logger.error("Download failed for object %s", sel)
                return
            self.layout.objaverse_items[sel] = ObjaverseItem(
                uid=item.uid, name=item.name, path=mesh_path
            )

        mesh_path = Path(mesh_path).resolve()
        mesh_path = convert_to_mujoco_mesh(mesh_path, out_ext=".obj")

        scale = self.layout.objaverse_scale.value
        parent_name = self.get_selected_parent()
        self.controller.create_mesh(parent_name, mesh_path, scale)

    def on_objaverse_item_change(self, _evt: GuiEvent) -> None:
        sel = self.layout.objaverse_list.value
        if sel not in self.layout.objaverse_items:
            return
        item: ObjaverseItem = self.layout.objaverse_items[sel]
        self.layout.objaverse_scale.value = lookup_default_scale(
            item,
            cachier__overwrite_cache=True,  # pyright: ignore[reportCallIssue]
        )

    def on_objaverse_scale_change(self, _evt: GuiEvent) -> None:
        sel = self.layout.objaverse_list.value
        if sel not in self.layout.objaverse_items:
            return
        item: ObjaverseItem = self.layout.objaverse_items[sel]
        SCALE_OVERRIDES[item.uid] = self.layout.objaverse_scale.value

    def on_objaverse_category_change(self, _evt: GuiEvent) -> None:
        label = self.layout.objaverse_category_list.value
        if label == "— all —":
            obj_items = self.obj_inventory.list_by_labels(
                labels=None, limit_per_label=50
            )
        else:
            obj_items = self.obj_inventory.list_by_labels(
                labels=[label], limit_per_label=50
            )
        self.layout.objaverse_items = {m.name: m for m in obj_items}
        self.layout.update_objaverse_dropdown()

    def export_mujoco(self, evt: GuiEvent) -> None:
        out_path = Path(self.layout.export_path.value).expanduser()
        self.controller.export_scene(out_path)
        evt.client.add_notification(
            "Exported",
            f"Model exported to {out_path}",
            auto_close_seconds=3.0,
            loading=False,
        )

    def launch_mujoco_viewer(self, evt: GuiEvent) -> None:
        out_path = Path(self.layout.export_path.value).expanduser()
        out_path = out_path.with_suffix(".xml")
        if not out_path.is_file():
            logger.error("Invalid path for MuJoCo file %s", out_path)
            evt.client.add_notification(
                "Error",
                f"Invalid path for MuJoCo file {out_path}",
                auto_close_seconds=3.0,
                loading=False,
            )
            return
        try:
            subprocess.Popen(
                [sys.executable, "-m", "mujoco.viewer", f"--mjcf={out_path}"]
            )
            logger.info("Launching MuJoCo viewer for %s", out_path)
        except Exception as e:
            logger.error("Failed to launch MuJoCo viewer: %s", e)

    def add_camera(self, _evt: GuiEvent) -> None:
        camera_name = self.layout.camera_list.value
        self.controller.create_camera(camera_name)

    def add_robot(self, _evt: GuiEvent) -> None:
        sel = self.layout.robot_list.value
        self.controller.create_robot(sel)

    def update_element(self, _evt: GuiEvent) -> None:
        sel = self.layout.element_list.value
        color = self.layout.prop_color.value
        opacity = self.layout.prop_opacity.value

        # TODO revise
        if not self.layout.prop_box_dims.disabled:
            dim = self.layout.prop_box_dims.value
            size = [dim[0] / 2.0, dim[1] / 2.0, dim[2] / 2.0]
        elif not self.layout.prop_sphere_radius.disabled:
            size = [self.layout.prop_sphere_radius.value, 0.0]
        elif not self.layout.prop_cyl_radius.disabled:
            size = [
                self.layout.prop_cyl_radius.value,
                self.layout.prop_cyl_height.value / 2.0,
            ]
        else:
            size = None
        mass = self.layout.prop_element_mass.value
        self.controller.update_element(sel, color, opacity, size=size, mass=mass)

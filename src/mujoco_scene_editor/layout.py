from typing import Dict
from typing import Any
from typing import Tuple

import logging

import viser

from robits.core.config_manager import config_manager

from mujoco_scene_editor.gui.transform_panel import TransformPanel
from mujoco_scene_editor.utils import viser_utils

from mujoco_scene_editor.constants import NO_SELECTION
from mujoco_scene_editor.constants import DEFAULT_ASSET_DIR
from mujoco_scene_editor.constants import DEFAULT_EXPORT_TARGET


logger = logging.getLogger(__name__)


class SceneEditorLayout:
    def __init__(self):
        self.asset_items: Dict[
            str, Any
        ] = {}  # name -> ObjectModel for the drop down menu
        self.objaverse_items: Dict[
            str, Any
        ] = {}  # name -> ObjaverseItem for the drop down menu
        self.objaverse_labels = ()  # tuple of category labels

        # Server + scene
        self.server = self._init_server()
        self.reset()

        self._init_gui()

        self.server.on_client_connect(self.on_connect)
        self.server.on_client_disconnect(self.on_disconnect)
        self.is_running = True

    def _init_server(self) -> viser.ViserServer:
        """
        Initialize the server. Call this only once.
        """
        # Only allow viser locally as the scene editor can overwrite files locally
        server = viser.ViserServer(host="127.0.0.1", label="MuJoCo Scene Editor")
        server.gui.configure_theme(dark_mode=True, control_layout="collapsible")
        server.scene.set_up_direction("+z")
        return server

    def on_connect(self, client: viser.ClientHandle) -> None:
        logger.info("New client %s", client)
        client.add_notification(
            "Hello World.",
            """
            This is still work in progress. If you like to help feel free to send me an e-mail or open a PR.
            """,
            auto_close_seconds=8.0,
            loading=False,
        )

    def on_disconnect(self, client: viser.ClientHandle) -> None:
        logger.info("Client disconnected %s", client)

    def reset(self):
        """
        Clears the scene and adds basic items such as the light and grid.

        .. todo:: also add a coordinate system
        """
        if self.server is None:
            raise RuntimeError("Please initialize the server first.")

        server = self.server
        server.scene.reset()

        server.scene.add_grid(
            "/grid", width=10.0, height=10.0, plane="xy", section_size=1.0
        )
        server.scene.add_light_ambient(
            "/lights/ambient", color=(255, 255, 255), intensity=0.5
        )

        server.scene.add_light_directional(
            "/lights/key",
            color=(255, 255, 255),
            intensity=1.2,
            wxyz=(0.924, 0, 0.383, 0),  # 45 deg rotation around y
            position=(3.0, 4.0, 2.0),
            cast_shadow=True,
        )

        return server

    def _init_gui(self) -> None:
        """
        Setup gui elements
        """
        if not self.server:
            raise RuntimeError("Server needs to be initialized first.")

        self.element_list = self.server.gui.add_dropdown(
            "Elements", options=(NO_SELECTION,), initial_value=NO_SELECTION
        )

        # general controls
        with self.server.gui.add_folder("Controls", expand_by_default=False):
            self.btn_clear = self.server.gui.add_button("Clear scene")
            self.btn_quit = self.server.gui.add_button("Quit server")
            self.btn_undo = self.server.gui.add_button("Undo", disabled=True)
            self.btn_redo = self.server.gui.add_button("Redo", disabled=True)

            self.allow_mouse_select = self.server.gui.add_checkbox(
                "Allow mouse selection", initial_value=True
            )
            self.enable_gizmo_checkbox = self.server.gui.add_checkbox(
                "Interactive translation", initial_value=True
            )
            self.btn_delete_element = self.server.gui.add_button(
                "Delete selected element"
            )

        with self.server.gui.add_folder("Add Group", expand_by_default=False):
            self.txt_group_name = self.server.gui.add_text(
                "Group prefix", "group", disabled=True
            )
            self.btn_create_group = self.server.gui.add_button("Add group")

        with self.server.gui.add_folder(
            "Add Primitive Shapes", expand_by_default=False
        ):
            with self.server.gui.add_folder("Box"):
                self.box_dims = self.server.gui.add_vector3(
                    "Dimensions", initial_value=(0.1, 0.1, 0.1), step=0.05
                )
                self.box_color = self.server.gui.add_rgb(
                    "Color", initial_value=(200, 100, 240)
                )
                self.box_opacity = self.server.gui.add_slider(
                    "Opacity", 0.0, 1.0, 0.01, 1.0
                )
                self.btn_create_box = self.server.gui.add_button("Create Box")

            with self.server.gui.add_folder("Cylinder"):
                self.cyl_radius = self.server.gui.add_slider(
                    "Radius", 0.01, 1.0, 0.01, 0.05
                )
                self.cyl_height = self.server.gui.add_slider(
                    "Height", 0.02, 2.0, 0.02, 0.2
                )
                self.cyl_color = self.server.gui.add_rgb(
                    "Color", initial_value=(200, 200, 80)
                )
                self.cyl_opacity = self.server.gui.add_slider(
                    "Opacity", 0.0, 1.0, 0.01, 1.0
                )
                self.btn_create_cylinder = self.server.gui.add_button("Create Cylinder")

            with self.server.gui.add_folder("Sphere"):
                self.sphere_radius = self.server.gui.add_slider(
                    "Radius", 0.01, 1.0, 0.01, 0.05
                )
                self.sphere_color = self.server.gui.add_rgb(
                    "Color", initial_value=(100, 180, 240)
                )
                self.sphere_opacity = self.server.gui.add_slider(
                    "Opacity", 0.0, 1.0, 0.01, 1.0
                )
                self.btn_create_sphere = self.server.gui.add_button("Create Sphere")

        with self.server.gui.add_folder(
            "Add Assets from File", expand_by_default=False
        ):
            # Keep as text input; no file picker available. Expanduser at use.
            self.assets_dir = self.server.gui.add_text(
                "Directory", initial_value=DEFAULT_ASSET_DIR
            )
            self.btn_scan_assets = self.server.gui.add_button("Scan assets")
            self.assets_list = self.server.gui.add_dropdown(
                "Items", options=(NO_SELECTION,), initial_value=NO_SELECTION
            )
            self.btn_add_asset = self.server.gui.add_button("Add asset")

        with self.server.gui.add_folder(
            "Add Assets from Objaverse", expand_by_default=False
        ):
            # Objaverse objects (from local cache)
            self.objaverse_category_list = self.server.gui.add_dropdown(
                "Category", options=("— all —",), initial_value="— all —"
            )
            self.objaverse_list = self.server.gui.add_dropdown(
                "Object", options=(NO_SELECTION,), initial_value=NO_SELECTION
            )
            self.objaverse_scale = self.server.gui.add_slider(
                "Unit scaling", 0.0001, 10.0, 0.0001, 0.01
            )
            self.btn_add_objaverse = self.server.gui.add_button("Add object")

        with self.server.gui.add_folder("Add Robot", expand_by_default=False):
            robot_choices = self.get_robot_choices()
            self.robot_list = self.server.gui.add_dropdown(
                "Robot", options=robot_choices
            )
            self.btn_add_robot = self.server.gui.add_button("Add robot")

        with self.server.gui.add_folder("Add Camera", expand_by_default=False):
            camera_choices = self.get_camera_choices()
            self.camera_list = self.server.gui.add_dropdown(
                "Camera", options=camera_choices
            )
            self.btn_add_camera = self.server.gui.add_button("Add camera")

        with self.server.gui.add_folder("Transform"):
            self.transform = TransformPanel(self.server)

        with self.server.gui.add_folder("Properties"):
            self.prop_color = self.server.gui.add_rgb(
                "Color",
                initial_value=(200, 200, 200),
            )
            self.prop_opacity = self.server.gui.add_slider(
                "Opacity", 0.0, 1.0, 0.01, 1.0
            )
            self.prop_box_dims = self.server.gui.add_vector3(
                "Box dimensions", initial_value=(0.1, 0.1, 0.1), step=0.01
            )
            self.prop_sphere_radius = self.server.gui.add_slider(
                "Sphere radius", 0.01, 2.0, 0.01, 0.05
            )
            self.prop_cyl_radius = self.server.gui.add_slider(
                "Cylinder radius", 0.01, 2.0, 0.01, 0.05
            )
            self.prop_cyl_height = self.server.gui.add_slider(
                "Cylinder height", 0.02, 4.0, 0.02, 0.2
            )
            self.btn_update_element = self.server.gui.add_button("Update element")

        with self.server.gui.add_folder("Export"):
            self.export_path = self.server.gui.add_text(
                "File", initial_value=DEFAULT_EXPORT_TARGET
            )
            self.btn_export_mj = self.server.gui.add_button("Export scene")
            self.btn_launch_mj = self.server.gui.add_button("Launch MuJoCo")

        self.gizmo = self.server.scene.add_transform_controls(
            "/transform", scale=0.8, visible=False
        )  # We can also set this to None

    def disable_all_properties_gui_elements(self):
        self.prop_color.disabled = True
        self.prop_opacity.disabled = True
        self.prop_box_dims.disabled = True
        self.prop_sphere_radius.disabled = True
        self.prop_cyl_radius.disabled = True
        self.prop_cyl_height.disabled = True

    def update_assets_dropdown(self) -> None:
        names = tuple(self.asset_items.keys()) or (NO_SELECTION,)
        self.assets_list.options = names
        self.assets_list.value = names[0]

    def update_objaverse_dropdown(self) -> None:
        names = tuple(self.objaverse_items.keys()) or (NO_SELECTION,)
        self.objaverse_list.options = names
        self.objaverse_list.value = names[0]

    def update_objaverse_label_dropdown(self) -> None:
        labels = getattr(self, "objaverse_labels", ())
        options = tuple(labels) or (NO_SELECTION,)
        self.objaverse_category_list.options = options
        self.objaverse_category_list.value = options[0]

    def get_robot_choices(self) -> Tuple[str, ...]:
        # supported_robots = {''}
        # robot_choices = [r for r in config_manager.available_robots if r in supported_robots]
        robot_choices = config_manager.available_robots
        return tuple(robot_choices) or (NO_SELECTION,)

    def get_camera_choices(self) -> Tuple[str, ...]:
        """
        .. todo:: filter
        """
        camera_choices = config_manager.available_cameras
        return tuple(camera_choices) or (NO_SELECTION,)

    def on_select(self, node) -> None:
        self.transform.on_select(node)

        if self.gizmo:
            logger.info("Removing previous gizmo %s", self.gizmo.name)
            self.gizmo.remove()

        # self.properties.on_select(node)
        gizmo_visible = self.enable_gizmo_checkbox.value

        parent_node_name = viser_utils.parent_name(node.name)
        base_node_name = viser_utils.base_name(node.name)

        gizmo = self.server.scene.add_transform_controls(
            f"{parent_node_name}/{base_node_name}_transform",
            visible=gizmo_visible,
            position=node.position,
            wxyz=node.wxyz,
        )

        @gizmo.on_update
        def _(_evt: viser.GuiEvent) -> None:
            node.position = gizmo.position
            node.wxyz = gizmo.wxyz

        self.gizmo = gizmo

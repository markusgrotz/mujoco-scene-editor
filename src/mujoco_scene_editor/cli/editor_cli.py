#!/usr/bin/env python

from typing import List
from typing import Optional
from functools import wraps

import logging
import os
import time
from pathlib import Path
import webbrowser

import rich_click as click
from rich.progress import Progress
from click_prompt import filepath_option
from click_prompt import filepath_argument
from click_prompt import input_text_argument

from robits.sim.blueprints import blueprints_from_json
from robits.sim.blueprints import Blueprint
from robits.sim.blueprints import GeomBlueprint
from robits.sim.blueprints import CameraBlueprint
from robits.sim.blueprints import Pose


from robits.utils import camera_intrinsics

from robits.cli.cli_utils import setup_cli

from mujoco_scene_editor.layout import SceneEditorLayout
from mujoco_scene_editor.scene_renderer import ViserSceneRenderer
from mujoco_scene_editor.controller import SceneEditorController
from mujoco_scene_editor.scene_editor import SceneEditor

from mujoco_scene_editor.constants import DEFAULT_ASSET_DIR
from mujoco_scene_editor.constants import DEFAULT_EXPORT_TARGET

logger = logging.getLogger(__name__)

setup_cli(logging.INFO)


def get_scene_editor(blueprints: Optional[List[Blueprint]] = None) -> SceneEditor:
    layout = SceneEditorLayout()
    renderer = ViserSceneRenderer(layout)
    controller = SceneEditorController(renderer)
    if blueprints:
        controller.load_blueprints(blueprints)

    return SceneEditor(controller, layout)


@click.group()
def cli():
    pass


@cli.command()
@click.option("--open-browser/--skip-open-browser", is_flag=True, default=True)
def new(open_browser: bool):
    """
    Start a new, empty scene.
    """
    default_blueprints: List[Blueprint] = [
        GeomBlueprint(
            "/floor",
            geom_type="plane",
            size=[2.5, 2.5, 0.01],
            rgba=[0.5, 0.5, 0.5, 1.0],
            pose=Pose().with_position([0, 0, -0.01]),
            is_static=True,
        ),
        CameraBlueprint(
            "/camera",
            width=640,
            height=480,
            intrinsics=camera_intrinsics.intrinsics_from_fovy(0.785398, 640, 480),
            pose=Pose().with_position([1.0, 0, 1.5]),
        ),
    ]
    viewer = get_scene_editor(default_blueprints)
    viewer.show()

    if open_browser:
        webbrowser.open_new(viewer.url)

    wait_until_keypress(viewer)


@cli.command()
@filepath_argument("model-name", default=DEFAULT_EXPORT_TARGET)
@click.option("--open-browser/--skip-open-browser", is_flag=True, default=True)
def edit(model_name: Path, open_browser: bool):
    """
    Load a scene from JSON or try to convert it from a MJCF XML.
    """
    path = Path(model_name).expanduser()
    if not path.exists() or not path.is_file():
        raise FileNotFoundError(f"Model file not found: {path}")

    if path.suffix.lower() == ".xml":
        from robits.sim.converters.mujoco_importer import load_mjcf_as_blueprints

        blueprints = load_mjcf_as_blueprints(path)
    else:
        json_data = path.read_text(encoding="utf-8")
        blueprints = blueprints_from_json(json_data)

    viewer = get_scene_editor(blueprints)
    viewer.show()

    if open_browser:
        webbrowser.open_new(viewer.url)

    wait_until_keypress(viewer)


@cli.command()
@click.option(
    "--root",
    default=DEFAULT_ASSET_DIR,
    help="Root directory to scan for assets (obj, stl, ply, glb, gltf, usd)",
)
def list_assets(root: str):
    """
    List all available assets found under a directory.
    """

    from mujoco_scene_editor.inventory.local_assets import Inventory

    inv = Inventory()
    root_path = Path(root).expanduser()

    if not root_path.exists() or not root_path.is_dir():
        raise FileNotFoundError(
            f"Folder does not exist or is not a directory: {root_path}"
        )

    items = inv.list(root=root_path)

    if not items:
        click.echo(f"No assets found under {root_path.resolve()}.")
        return

    click.echo(f"Found {len(items)} assets under {root_path.resolve()}:")
    for m in items:
        click.echo(f"- {m.name}: {m.path}")


def validate_has_openai_key(func):
    @wraps(func)
    def _wrapper(*args, **kwargs):
        if not os.environ.get("OPENAI_API_KEY"):
            logger.error("Environment variable OPENAI_API_KEY is not set.")
            raise RuntimeError(
                "Missing OPENAI_API_KEY. Please export it with export OPENAI_API_KEY=... and retry."
            )
        return func(*args, **kwargs)

    return _wrapper


# @validate_has_openai_key
@cli.command()
@filepath_option(
    "--output-model-name",
    default=str(Path(DEFAULT_EXPORT_TARGET).with_name("scene_prompt.xml")),
)
@input_text_argument(
    "prompt", default="A detailed kitchen with a robot.", prompt="Describe your scene."
)
def prompt(output_model_name: str, prompt: str) -> None:
    """
    Ask ChatGPT to generate a scene. Requires an OpenAI API key
    """
    from robits.vlm.openai_vlm import PromptBuilder
    from robits.vlm.openai_vlm import ChatGPT
    import re

    prefix = """
Generate a MuJoCo 3.3.7 XML. Here are some guidelines:
- Don't use global tags for colors or other elements
- Don't use textures/materials. Use a color instead assigned to each element
- If the scene should contain a robot try match it first to an existing description in the robot_description package (https://github.com/robot-descriptions/robot_descriptions.py). If found copy the content.
- For moving objects add a freejoint
- There is no geom cone element
- Avoid accelerometer and sensor tags
Output the complete XML file. The scene is as follows:
"""
    chatgpt = ChatGPT()

    builder = PromptBuilder()
    builder.add_instruction(prefix)
    builder.add_instruction(prompt)

    with Progress() as progress:
        progress.add_task("Querying.", total=None)
        response = chatgpt.query(builder)

    # click.echo(response)

    content = response.choices[0].message.content

    pattern = r"<mujoco\b[^>]*>.*?</mujoco>"
    match = re.search(pattern, content, flags=re.DOTALL)

    if match is None:
        click.echo(response)
        logger.error("Unable to parse response")
        return

    xml = match.group(0)

    output_path = Path(output_model_name).expanduser()

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(xml)

    click.echo(f"Edit the model with mjedit --model-name {output_path}")


def wait_until_keypress(viewer) -> None:
    try:
        while viewer.is_running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt.")
        viewer.quit_server(None)


if __name__ == "__main__":
    cli()

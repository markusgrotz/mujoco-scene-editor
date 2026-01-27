# Scene Editor

[![Supported Python Versions](https://img.shields.io/pypi/pyversions/mujoco-scene-editor)](https://pypi.org/project/mujoco-scene-editor/)
[![PyPI version](https://img.shields.io/pypi/v/mujoco-scene-editor)](https://pypi.org/project/mujoco-scene-editor/)
[![License](https://img.shields.io/pypi/l/mujoco-scene-editor)](https://github.com/markusgrotz/mujoco-scene-editor/blob/main/LICENSE.md)
[![Code style](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)


Lightweight, interactive scene editor for MuJoCo 3.x. Create or edit scenes in
your browser to place shapes, import meshes, add robots, and edit elements interactively.

## Install

From a local checkout:

```bash
pip install -e .
# or with dev tools
pip install -e '.[dev]'
```

Without installing, you can also run with uv:

```bash
# Run the installed console script via uv
uv run scene_editor --help
uv run scene_editor new
```

The following entry points do a exit, which are part of the `scene_editor` sub commands:
 - mjcreate
 - mjedit
 - mjprompt


## Quickstart

```bash
# Start a fresh, empty scene
mjcreate

# Load from MJCF XML or a blueprint JSON
mjedit path/to/scene.xml

# Scan a directory for supported assets (obj, stl, ply, glb, gltf, usd)
scene_editor list-assets --root ~/path/to/assets
```

When the server starts, it prints a local URL and opens your browser. Quit with Ctrl+C or the "Quit server" button.

## Example

Use the provided chemistry lab MJCF as a starting point:

```bash
# With pip-installed package
mjedit examples/prompt/scene_chemistry_lab.xml

# With uv (no install)
uv run mjedit examples/prompt/scene_chemistry_lab.xml
```

Then:
- Use "Add Box/Sphere/Cylinder" to place primitive geoms.
- Use "Add Asset" to insert a local mesh from your file system.
- Drag the gizmo to change pose; use “Export” to write MJCF/JSON.


## Prompting / Scene Generation Examples

You can conveniently generate a MuJoCo scene from a natural-language prompt (requires an OpenAI API key):

```bash
# Set this to your API key
export OPENAI_API_KEY=...
# Generate a scene from a prompt string
mjprompt

# Edit the generated scene. 
mjedit examples/prompt/scene_coffee_shop.xml
```
Loading a generated scene might not work out of the box in all cases. Generated scenes can have inconsistencies in geometry but can be easily edited.
### Examples

Below are some generated example scenes. More examples are available in the `examples/prompt` folder.
 
 <table>
   <tr>
     <td align="center">
      <a href="https://raw.githubusercontent.com/markusgrotz/mujoco-scene-editor/main/assets/prompt/scene_living_room_large.png">
        <img src="https://raw.githubusercontent.com/markusgrotz/mujoco-scene-editor/main/assets/prompt/scene_living_room_small.png" alt="Living Room" width="420" />
      </a><br/><sub>Living Room</sub>
     </td>
     <td align="center">
      <a href="https://raw.githubusercontent.com/markusgrotz/mujoco-scene-editor/main/assets/prompt/scene_chess_large.png">
        <img src="https://raw.githubusercontent.com/markusgrotz/mujoco-scene-editor/main/assets/prompt/scene_chess_small.png" alt="Chess Table" width="420" />
      </a><br/><sub>Chess Table</sub>
     </td>
   </tr>
   <tr>
     <td align="center">
      <a href="https://raw.githubusercontent.com/markusgrotz/mujoco-scene-editor/main/assets/prompt/scene_playground_large.png">
        <img src="https://raw.githubusercontent.com/markusgrotz/mujoco-scene-editor/main/assets/prompt/scene_playground_small.png" alt="Playground" width="420" />
      </a><br/><sub>Playground</sub>
     </td>
     <td align="center">
      <a href="https://raw.githubusercontent.com/markusgrotz/mujoco-scene-editor/main/assets/prompt/scene_chemistry_lab_large.png">
        <img src="https://raw.githubusercontent.com/markusgrotz/mujoco-scene-editor/main/assets/prompt/scene_chemistry_lab_small.png" alt="Chemistry Lab" width="420" />
      </a><br/><sub>Chemistry Lab</sub>
     </td>
   </tr>
 </table>

## Limitations

- Importing MuJoCo XML files may alter the internal structure, and some tags are discarded.
- Not all MuJoCo robot descriptions have an equivalent URDF representation.

## Links

- Repository: https://github.com/markusgrotz/mujoco-scene-editor
- Issues: https://github.com/markusgrotz/mujoco-scene-editor/issues

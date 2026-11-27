"""Convert the supplied FBX deformation skeleton to a Bevy-loadable GLB.

This script is run by GitHub Actions. It deliberately exports no animation clips:
the human action JSON files are motion-goal inputs to the physics controller, not
skeletal animation clips."""
from pathlib import Path
import bpy


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets" / "human" / "Character_Rig.fbx"
OUTPUT = ROOT / "assets" / "human" / "Character_Rig.glb"


def main() -> None:
    if not SOURCE.is_file():
        raise FileNotFoundError(f"Missing source character model: {SOURCE}")

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=str(SOURCE), use_anim=False)

    meshes = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    armatures = [obj for obj in bpy.context.scene.objects if obj.type == "ARMATURE"]
    if not meshes:
        raise RuntimeError("FBX import produced no mesh objects")
    if not armatures:
        raise RuntimeError("FBX import produced no deformation armature")

    # Ensure the complete imported mesh + skin hierarchy is exported.
    bpy.ops.object.select_all(action="DESELECT")
    for obj in meshes + armatures:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = armatures[0]

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(OUTPUT),
        export_format="GLB",
        export_skins=True,
        export_animations=False,
        export_yup=True,
    )

    if not OUTPUT.is_file() or OUTPUT.stat().st_size <= 0:
        raise RuntimeError("glTF export did not create a non-empty GLB file")


if __name__ == "__main__":
    main()

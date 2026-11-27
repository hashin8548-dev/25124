"""Convert the supplied FBX deformation skeleton to a Bevy-loadable GLB.

This script runs in GitHub Actions. It deliberately disables FBX animation import and
glTF animation export: human action JSON files are physics-motion inputs, not clips.
"""
from pathlib import Path
import bpy


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets" / "human" / "Character_Rig.fbx"
OUTPUT = ROOT / "assets" / "human" / "Character_Rig.glb"


def main() -> None:
    if not SOURCE.is_file():
        raise FileNotFoundError(f"Missing source character model: {SOURCE}")

    bpy.ops.wm.read_factory_settings(use_empty=True)

    # Blender 5.x uses the built-in ufbx importer under bpy.ops.wm.fbx_import.
    # Inspect its RNA contract so an API change fails before an accidental
    # animation import or silent fallback to a different importer.
    importer = bpy.ops.wm.fbx_import
    supported = {prop.identifier for prop in importer.get_rna_type().properties}
    required = {"filepath", "use_anim"}
    missing = required - supported
    if missing:
        raise RuntimeError(f"Blender FBX importer lacks required no-animation options: {sorted(missing)}")
    importer(filepath=str(SOURCE), use_anim=False)

    meshes = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    armatures = [obj for obj in bpy.context.scene.objects if obj.type == "ARMATURE"]
    if not meshes:
        raise RuntimeError("FBX import produced no mesh objects")
    if not armatures:
        raise RuntimeError("FBX import produced no deformation armature")

    # FBX source is the static deformation skeleton; export only mesh + skin.
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

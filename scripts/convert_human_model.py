"""Build a Bevy-ready GLB and rig metadata from the static deformation FBX.

The source action JSON files are body-motion targets. This converter deliberately
imports and exports no animation clips; it also exports the deformation skeleton's
rest hierarchy and geometry so Rust can build a dynamic articulated ragdoll.
"""
from __future__ import annotations

import json
from pathlib import Path
import bpy
from mathutils import Matrix


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets" / "human" / "Character_Rig.fbx"
OUTPUT = ROOT / "assets" / "human" / "Character_Rig.glb"
HUMAN_MANIFEST = ROOT / "data" / "human" / "human.json"
RIG_OUTPUT = ROOT / "data" / "human" / "rig.json"
IDLE_ACTION = ROOT / "data" / "human" / "actions" / "idle.json"


def matrix_rows(matrix: Matrix) -> list[list[float]]:
    return [[float(matrix[row][column]) for column in range(4)] for row in range(4)]


def vector3(vector) -> list[float]:
    return [float(vector.x), float(vector.y), float(vector.z)]


def matrix_error(left: Matrix, right: Matrix) -> float:
    return sum(
        (float(left[row][column]) - float(right[row][column])) ** 2
        for row in range(4)
        for column in range(4)
    ) ** 0.5


def load_manifest() -> dict:
    with HUMAN_MANIFEST.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    if manifest.get("schema_version") != 1:
        raise RuntimeError("Unsupported human manifest schema version")
    rig = manifest.get("rig", {})
    body_tracks = rig.get("body_tracks")
    marker_tracks = rig.get("joint_marker_tracks")
    anchor_tracks = rig.get("joint_anchor_tracks")
    if not isinstance(body_tracks, list) or not body_tracks:
        raise RuntimeError("human.json must define rig.body_tracks")
    if not isinstance(marker_tracks, list) or not marker_tracks:
        raise RuntimeError("human.json must define rig.joint_marker_tracks")
    if not isinstance(anchor_tracks, dict) or not anchor_tracks:
        raise RuntimeError("human.json must define rig.joint_anchor_tracks")
    if (
        not all(isinstance(body, str) and body in body_tracks for body in anchor_tracks)
        or not all(isinstance(marker, str) and marker in marker_tracks for marker in anchor_tracks.values())
        or set(anchor_tracks.values()) != set(marker_tracks)
        or len(set(anchor_tracks.values())) != len(anchor_tracks)
    ):
        raise RuntimeError("human.json joint_anchor_tracks must map body tracks one-to-one to all joint markers")
    return manifest



def extract_weighted_collision_geometry(
    armature: bpy.types.Object,
    meshes: list[bpy.types.Object],
    body_track_names: set[str],
) -> dict[str, dict]:
    """Measure each body's collision envelope from weighted source mesh vertices."""
    weighted_vertices: dict[str, list] = {name: [] for name in body_track_names}
    armature_inverse = armature.matrix_world.inverted_safe()

    for mesh in meshes:
        mesh_to_armature = armature_inverse @ mesh.matrix_world
        group_names = {group.index: group.name for group in mesh.vertex_groups}
        for vertex in mesh.data.vertices:
            memberships = [
                membership
                for membership in vertex.groups
                if membership.weight >= 0.5
                and group_names.get(membership.group) in body_track_names
            ]
            if not memberships:
                continue
            position_armature = mesh_to_armature @ vertex.co
            for membership in memberships:
                name = group_names[membership.group]
                bone = armature.data.bones.get(name)
                if bone is None:
                    continue
                position_bone = bone.matrix_local.inverted_safe() @ position_armature
                weighted_vertices[name].append(position_bone.copy())

    # The FBX root is a non-deforming pelvis control bone, so it has no direct
    # vertex-group weights. Derive its envelope from the real weighted torso and
    # thigh vertices in root-bone local coordinates instead of inventing dimensions.
    root_points = weighted_vertices.get("Root", [])
    if len(root_points) < 4:
        root_bone = armature.data.bones.get("Root")
        if root_bone is None:
            raise RuntimeError("The human rig has no Root bone for pelvis collision geometry")
        root_inverse = root_bone.matrix_local.inverted_safe()
        source_groups = {"Torso", "Thigh_L", "Thigh_R"}
        for mesh in meshes:
            mesh_to_armature = armature_inverse @ mesh.matrix_world
            group_names = {group.index: group.name for group in mesh.vertex_groups}
            for vertex in mesh.data.vertices:
                position_armature = mesh_to_armature @ vertex.co
                for membership in vertex.groups:
                    if (
                        membership.weight >= 0.35
                        and group_names.get(membership.group) in source_groups
                    ):
                        root_points.append(root_inverse @ position_armature)

    collision = {}
    missing = []
    for name, points in weighted_vertices.items():
        if name == "Root":
            points = root_points
        if len(points) < 4:
            missing.append(name)
            continue
        radial = [((point.x ** 2) + (point.z ** 2)) ** 0.5 for point in points]
        axial = [float(point.y) for point in points]
        radius = max(radial)
        axial_min = min(axial)
        axial_max = max(axial)
        axial_length = axial_max - axial_min
        if not all(map(lambda value: value == value and abs(value) != float("inf"),
                       [radius, axial_min, axial_max, axial_length])):
            raise RuntimeError(f"Non-finite weighted collision geometry for bone {name}")
        if radius <= 1.0e-4 or axial_length <= 1.0e-4:
            missing.append(name)
            continue
        collision[name] = {
            "shape": "capsule",
            "radius_m": float(radius),
            "axial_min_m": float(axial_min),
            "axial_max_m": float(axial_max),
            "axial_length_m": float(axial_length),
            "capsule_segment_length_m": float(max(0.0, axial_length - 2.0 * radius)),
            "center_offset_bone_m": [0.0, float((axial_min + axial_max) * 0.5), 0.0],
            "weighted_vertex_count": len(points),
            "weight_threshold": 0.5,
        }

    if missing:
        raise RuntimeError(
            "Could not derive collision envelopes from weighted mesh vertices for body tracks: "
            + ", ".join(sorted(set(missing)))
        )
    return collision


def extract_rig(
    armature: bpy.types.Object,
    manifest: dict,
    meshes: list[bpy.types.Object],
) -> dict:
    body_tracks = set(manifest["rig"]["body_tracks"])
    marker_tracks = set(manifest["rig"]["joint_marker_tracks"])
    bones = armature.data.bones
    bone_names = {bone.name for bone in bones}

    missing_body_tracks = sorted(body_tracks - bone_names)
    if missing_body_tracks:
        raise RuntimeError(
            "Action body tracks have no matching deformation bones: "
            + ", ".join(missing_body_tracks)
        )

    missing_marker_tracks = sorted(marker_tracks - bone_names)
    if missing_marker_tracks:
        print(
            "WARNING: joint marker tracks without deformation bones: "
            + ", ".join(missing_marker_tracks)
        )

    armature_world = armature.matrix_world.copy()
    collision_geometry = extract_weighted_collision_geometry(armature, meshes, body_tracks)
    if not IDLE_ACTION.is_file():
        raise FileNotFoundError(f"Missing neutral reference action: {IDLE_ACTION}")
    with IDLE_ACTION.open("r", encoding="utf-8") as stream:
        idle = json.load(stream)
    idle_tracks = {track["name"]: track for track in idle.get("animation", [])}
    records = []
    for bone in sorted(bones, key=lambda item: item.name):
        records.append(
            {
                "name": bone.name,
                "parent": bone.parent.name if bone.parent else None,
                "deform": bool(bone.use_deform),
                "length_m": float(bone.length),
                "head_armature_m": vector3(bone.head_local),
                "tail_armature_m": vector3(bone.tail_local),
                "matrix_armature": matrix_rows(bone.matrix_local),
                "matrix_parent_local": matrix_rows(
                    bone.parent.matrix_local.inverted_safe() @ bone.matrix_local
                    if bone.parent
                    else bone.matrix_local
                ),
                "matrix_world": matrix_rows(armature_world @ bone.matrix_local),
                "action_reference_parent_local": (
                    idle_tracks[bone.name]["transform"][0]
                    if bone.name in idle_tracks
                    else None
                ),
                "collision_geometry": collision_geometry.get(bone.name),
                "body_track": bone.name if bone.name in body_tracks else None,
                "joint_marker_track": bone.name if bone.name in marker_tracks else None,
                "joint_anchor_track": manifest["rig"]["joint_anchor_tracks"].get(bone.name),
            }
        )

    pose_bones = armature.pose.bones
    errors = {
        "parent_relative_rest": [],
        "bone_local_basis": [],
        "armature_rest": [],
        "armature_pose": [],
    }
    for track in idle.get("animation", []):
        name = track.get("name")
        pose_bone = pose_bones.get(name)
        samples = track.get("transform")
        if pose_bone is None or not samples:
            continue
        source_matrix = Matrix(
            [samples[0][0:4], samples[0][4:8], samples[0][8:12], samples[0][12:16]]
        )
        parent_relative_rest = (
            pose_bone.parent.bone.matrix_local.inverted_safe() @ pose_bone.bone.matrix_local
            if pose_bone.parent
            else pose_bone.bone.matrix_local
        )
        errors["parent_relative_rest"].append(matrix_error(source_matrix, parent_relative_rest))
        errors["bone_local_basis"].append(matrix_error(source_matrix, pose_bone.matrix_basis))
        errors["armature_rest"].append(matrix_error(source_matrix, pose_bone.bone.matrix_local))
        errors["armature_pose"].append(matrix_error(source_matrix, pose_bone.matrix))

    mean_errors = {
        key: (sum(values) / len(values) if values else None)
        for key, values in errors.items()
    }
    print(f"Human action/rest-space matrix comparison: {mean_errors}")

    return {
        "schema_version": 1,
        "source_model": "human/Character_Rig.fbx",
        "converted_model": "human/Character_Rig.glb",
        "coordinate_system": {
            "source_up": "+Z",
            "source_front": "+Y",
            "engine_up": "+Y",
            "engine_forward": "-Z",
            "conversion": "source_to_engine_rotation_x_minus_90_degrees",
        },
        "armature": {
            "name": armature.name,
            "matrix_world": matrix_rows(armature_world),
            "action_matrix_space_comparison_mean_error": mean_errors,
        },
        "body_track_names": sorted(body_tracks),
        "joint_marker_track_names": sorted(marker_tracks),
        "bones": records,
    }


def main() -> None:
    if not SOURCE.is_file():
        raise FileNotFoundError(f"Missing source character model: {SOURCE}")

    manifest = load_manifest()
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # Blender 5.x exposes the built-in ufbx importer as bpy.ops.wm.fbx_import.
    importer = bpy.ops.wm.fbx_import
    supported = {prop.identifier for prop in importer.get_rna_type().properties}
    required = {"filepath", "use_anim"}
    missing_options = required - supported
    if missing_options:
        raise RuntimeError(
            f"Blender FBX importer lacks required no-animation options: {sorted(missing_options)}"
        )
    importer(filepath=str(SOURCE), use_anim=False)

    meshes = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    armatures = [obj for obj in bpy.context.scene.objects if obj.type == "ARMATURE"]
    if not meshes:
        raise RuntimeError("FBX import produced no mesh objects")
    deform_armatures = [
        obj for obj in armatures if any(bone.use_deform for bone in obj.data.bones)
    ]
    if len(deform_armatures) != 1:
        raise RuntimeError(
            f"Expected exactly one deformation armature, found {len(deform_armatures)}"
        )
    armature = deform_armatures[0]
    rig_metadata = extract_rig(armature, manifest, meshes)

    bpy.ops.object.select_all(action="DESELECT")
    for obj in meshes + armatures:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = armature

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    RIG_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(OUTPUT),
        export_format="GLB",
        export_skins=True,
        export_animations=False,
        export_yup=True,
    )
    if not OUTPUT.is_file() or OUTPUT.stat().st_size <= 0:
        raise RuntimeError("glTF export did not create a non-empty GLB file")

    with RIG_OUTPUT.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(rig_metadata, stream, indent=2)
        stream.write("\n")
    if not RIG_OUTPUT.is_file() or RIG_OUTPUT.stat().st_size <= 0:
        raise RuntimeError("Rig extraction did not create a non-empty rig.json file")

    print(
        f"Exported {len(meshes)} mesh objects, {len(rig_metadata['bones'])} deformation bones, "
        f"{len(rig_metadata['body_track_names'])} mapped body tracks and "
        f"{len(rig_metadata['joint_marker_track_names'])} joint marker tracks."
    )


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        raise SystemExit(1)

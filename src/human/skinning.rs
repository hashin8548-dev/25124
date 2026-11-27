use std::collections::{BTreeMap, BTreeSet, HashMap};

use bevy::prelude::*;

use crate::world::WorldOrigin;

use super::{
    HumanActionState, HumanBodyPart, HumanMotionLibrary, HumanRig, HumanSkinnedSceneRoot,
};

/// Mirrors solver-owned body transforms into the actual skinned GLB hierarchy.
/// Physics segments are authoritative for body bones; helper/tool bones use calibrated
/// source motion targets so the mesh stays visually coherent between constrained segments.
pub fn sync_human_skinned_mesh(
    origin: Res<WorldOrigin>,
    actions: Res<HumanMotionLibrary>,
    action_state: Res<HumanActionState>,
    rig: Res<HumanRig>,
    scene_roots: Query<&Children, With<HumanSkinnedSceneRoot>>,
    children: Query<&Children>,
    mut named_nodes: Query<(
        Entity,
        &Name,
        &mut Transform,
        Option<&ChildOf>,
        Option<&HumanBodyPart>,
    )>,
    global_transforms: Query<&GlobalTransform>,
) {
    let Some(action) = actions.action(&action_state.action_name) else {
        return;
    };
    let source_pose = match rig.sample_source_world_pose(
        action,
        action_state.elapsed_seconds,
        action_state.reverse,
    ) {
        Ok(pose) => pose,
        Err(error) => {
            warn!("Human skin target sampling failed: {error:#}");
            return;
        }
    };

    // Only visit nodes under this character's tagged GLB WorldAssetRoot instance.
    let mut scene_entities = BTreeSet::new();
    for root_children in &scene_roots {
        let mut pending: Vec<Entity> = root_children.iter().copied().collect();
        while let Some(entity) = pending.pop() {
            if !scene_entities.insert(entity) {
                continue;
            }
            if let Ok(descendants) = children.get(entity) {
                pending.extend(descendants.iter().copied());
            }
        }
    }
    if scene_entities.is_empty() {
        // The GLB world instance has not spawned its hierarchy yet.
        return;
    }

    let mut node_entities = HashMap::<String, Entity>::new();
    let mut physical_transforms = HashMap::<String, Transform>::new();
    for (entity, name, transform, _, body_part) in &mut named_nodes {
        if let Some(part) = body_part {
            physical_transforms.insert(part.bone_name.clone(), *transform);
        }
        if scene_entities.contains(&entity) && rig.bone(name.as_str()).is_some() {
            node_entities.insert(name.as_str().to_owned(), entity);
        }
    }

    let Some(root_transform) = physical_transforms.get("Root") else {
        return;
    };
    let Some(root_geometry) = rig
        .bone("Root")
        .and_then(|bone| bone.collision_geometry.as_ref())
    else {
        return;
    };

    let root_offset = Vec3::from_array(root_geometry.center_offset_bone_m);
    let root_head = root_transform.translation - root_transform.rotation * root_offset;
    let root_actual_world = Mat4::from_rotation_translation(root_transform.rotation, root_head);

    // Convert action/rest matrices from source Z-up to engine Y-up and then into the
    // current floating-origin coordinate frame used by the physics world.
    let source_to_engine =
        Mat4::from_quat(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2));
    let engine_to_source = source_to_engine.inverse();
    let origin_offset = origin.0.as_vec3();
    let convert_source_world = |mut matrix: Mat4| {
        matrix = source_to_engine * matrix * engine_to_source;
        matrix.w_axis = (matrix.w_axis.truncate() - origin_offset).extend(1.0);
        matrix
    };

    let Some(root_target_source) = source_pose.get("Root") else {
        return;
    };
    let root_target_world = convert_source_world(*root_target_source);
    let root_delta = root_actual_world * root_target_world.inverse();

    let mut desired_world = BTreeMap::<String, Mat4>::new();
    for name in rig.bone_names() {
        if let Some(body_transform) = physical_transforms.get(name) {
            let Some(geometry) = rig
                .bone(name)
                .and_then(|bone| bone.collision_geometry.as_ref())
            else {
                continue;
            };
            let center_offset = Vec3::from_array(geometry.center_offset_bone_m);
            let bone_head =
                body_transform.translation - body_transform.rotation * center_offset;
            desired_world.insert(
                name.to_owned(),
                Mat4::from_rotation_translation(body_transform.rotation, bone_head),
            );
        } else if let Some(source_matrix) = source_pose.get(name) {
            desired_world.insert(
                name.to_owned(),
                root_delta * convert_source_world(*source_matrix),
            );
        }
    }

    // Compose local node transforms from the current desired world transforms. For root
    // bones, the armature object is the non-bone parent and supplies the live scene offset.
    for name in rig.bone_names() {
        let Some(&entity) = node_entities.get(name) else {
            continue;
        };
        let Some(target_world) = desired_world.get(name) else {
            continue;
        };
        let Some(bone) = rig.bone(name) else {
            continue;
        };

        let parent_world = if let Some(parent_name) = &bone.parent {
            let Some(parent_matrix) = desired_world.get(parent_name) else {
                continue;
            };
            *parent_matrix
        } else {
            let Ok((_, _, _, Some(child_of), _)) = named_nodes.get(entity) else {
                continue;
            };
            let Ok(parent_global) = global_transforms.get(child_of.parent()) else {
                continue;
            };
            parent_global.to_matrix()
        };

        let local = parent_world.inverse() * *target_world;
        if !local.to_cols_array().iter().all(|value| value.is_finite()) {
            continue;
        }
        if let Ok((_, _, mut transform, _, _)) = named_nodes.get_mut(entity) {
            *transform = Transform::from_matrix(local);
        }
    }
}

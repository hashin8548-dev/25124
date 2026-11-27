use std::collections::{BTreeMap, BTreeSet, HashMap};

use bevy::prelude::*;

use super::{HumanActionState, HumanBodyPart, HumanMotionLibrary, HumanRig, HumanSkinnedSceneRoot};

/// Mirrors solver-owned body transforms into the actual skinned GLB hierarchy.
/// Physical bones follow their rigid bodies; helper/tool bones retain calibrated local
/// action targets relative to their current physical parent.
pub fn sync_human_skinned_mesh(
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
    let source_local_pose = match rig.sample_calibrated_local_pose(
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
        // WorldAssetRoot has not finished creating its world instance yet.
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

    // Convert parent-relative action targets into the Y-up basis used by the GLB export.
    let source_to_engine =
        Mat4::from_quat(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2));
    let engine_to_source = source_to_engine.inverse();
    let mut desired_world = BTreeMap::<String, Mat4>::new();
    let mut pending: BTreeSet<String> = rig.bone_names().map(str::to_owned).collect();

    while !pending.is_empty() {
        let mut progressed = false;
        let names: Vec<_> = pending.iter().cloned().collect();
        for name in names {
            let Some(bone) = rig.bone(&name) else {
                continue;
            };
            let Some(&node_entity) = node_entities.get(&name) else {
                continue;
            };

            let parent_world = if let Some(parent_name) = &bone.parent {
                let Some(parent_matrix) = desired_world.get(parent_name) else {
                    continue;
                };
                *parent_matrix
            } else {
                let Ok((_, _, _, Some(child_of), _)) = named_nodes.get(node_entity) else {
                    continue;
                };
                let Ok(parent_global) = global_transforms.get(child_of.parent()) else {
                    continue;
                };
                parent_global.to_matrix()
            };

            let desired = if let Some(body_transform) = physical_transforms.get(&name) {
                let Some(geometry) = bone.collision_geometry.as_ref() else {
                    continue;
                };
                let center_offset = Vec3::from_array(geometry.center_offset_bone_m);
                let bone_head = body_transform.translation - body_transform.rotation * center_offset;
                Mat4::from_rotation_translation(body_transform.rotation, bone_head)
            } else {
                let Some(source_local) = source_local_pose.get(&name) else {
                    continue;
                };
                let engine_local = source_to_engine * *source_local * engine_to_source;
                parent_world * engine_local
            };

            if !desired.to_cols_array().iter().all(|value| value.is_finite()) {
                continue;
            }
            desired_world.insert(name.clone(), desired);
            pending.remove(&name);
            progressed = true;
        }
        if !progressed {
            // The GLB instance may still be spawning a parent node; retry next frame.
            break;
        }
    }

    // Convert solved world-space bone targets back to local GLB node transforms.
    for (name, entity) in node_entities {
        let Some(target_world) = desired_world.get(&name) else {
            continue;
        };
        let Some(bone) = rig.bone(&name) else {
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

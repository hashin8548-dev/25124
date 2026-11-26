use std::{
    collections::{HashMap, HashSet, VecDeque},
    fs,
};

use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::{
    prelude::*,
    tasks::{AsyncComputeTaskPool, Task, block_on, futures_lite::future::poll_once},
};

use crate::{
    camera::FreeFlyCamera,
    config::{ChunkFile, ChunkSource, RuntimeConfig, validate_chunk_file},
    persistence::{
        PersistRuntimeState, PersistedBodyState, PersistedFragment, PersistedFragmentState,
        PersistentObjectId, WorldStateFile,
    },
    physics::{DestructionJournal, VoxelBond},
    world::{ChunkMembership, SpawnedVoxelBody, spawn},
};

struct PendingChunk {
    source: ChunkSource,
    task: Task<Result<ChunkFile, String>>,
}

#[derive(Resource, Default)]
pub struct ChunkStreamingState {
    pending: HashMap<String, PendingChunk>,
    ready: VecDeque<(ChunkSource, ChunkFile)>,
    loaded: HashSet<String>,
    failed: HashSet<String>,
    pub(crate) voxel_bodies: HashMap<[i32; 3], SpawnedVoxelBody>,
    active_bond_ids: HashSet<String>,
}

pub(super) fn update_chunk_streaming(
    mut commands: Commands,
    config: Res<RuntimeConfig>,
    camera: Query<&Transform, With<FreeFlyCamera>>,
    mut state: ResMut<ChunkStreamingState>,
    mut world_state: ResMut<WorldStateFile>,
    journal: Res<DestructionJournal>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    bodies: Query<
        (
            &PersistentObjectId,
            &ChunkMembership,
            &Transform,
            &LinearVelocity,
            &AngularVelocity,
        ),
        With<PersistRuntimeState>,
    >,
    fragments: Query<(
        &PersistentObjectId,
        &PersistedFragment,
        &ChunkMembership,
        &Transform,
        &LinearVelocity,
        &AngularVelocity,
    )>,
    chunk_entities: Query<(Entity, &ChunkMembership)>,
    bonds: Query<(Entity, &VoxelBond)>,
) {
    state
        .active_bond_ids
        .retain(|id| !journal.broken_bonds.contains(id));
    state
        .voxel_bodies
        .retain(|_, body| chunk_entities.get(body.entity).is_ok());

    let camera_position = camera
        .iter()
        .next()
        .map(|transform| transform.translation)
        .unwrap_or_else(|| Vec3::from_array(config.engine.camera.spawn_position_m));

    // Poll all outstanding asynchronous reads. Completed payloads are validated before
    // entering the spawn queue, so malformed/untrusted chunk data never creates entities.
    let mut completed = Vec::new();
    for (id, pending) in &mut state.pending {
        if let Some(result) = block_on(poll_once(&mut pending.task)) {
            completed.push((id.clone(), result));
        }
    }
    for (id, result) in completed {
        let Some(pending) = state.pending.remove(&id) else {
            continue;
        };
        match result {
            Ok(chunk) => match validate_chunk_file(&chunk, &pending.source, &config.components) {
                Ok(()) => state.ready.push_back((pending.source, chunk)),
                Err(error) => {
                    state.failed.insert(id.clone());
                    error!("Rejected streamed chunk '{}': {error:#}", id);
                },
            },
            Err(error) => {
                state.failed.insert(id.clone());
                error!("Could not read streamed chunk '{}': {error}", id);
            },
        }
    }

    // Cancel reads that are no longer relevant. Dropping the task drops its future; no
    // chunk is allowed to become resident simply because it was requested earlier.
    let load_limit = config.engine.streaming.load_radius_chunks as f32;
    let unload_limit = config.engine.streaming.unload_radius_chunks as f32;
    let cancelled: Vec<String> = state
        .pending
        .iter()
        .filter(|(_, pending)| {
            distance_to_chunk_aabb(camera_position, &pending.source)
                > chunk_radius_m(&pending.source, load_limit)
        })
        .map(|(id, _)| id.clone())
        .collect();
    for id in cancelled {
        state.pending.remove(&id);
    }

    // Unload chunks with hysteresis. Capture all runtime state before despawning entities,
    // so streaming never loses a dynamic body or a detached fragment.
    let loaded_to_unload: Vec<String> = state
        .loaded
        .iter()
        .filter_map(|id| {
            let source = config.chunks.iter().find(|source| &source.id == id)?;
            let unload_distance = chunk_radius_m(source, unload_limit);
            let chunk_is_distant =
                distance_to_chunk_aabb(camera_position, source) > unload_distance;
            if !chunk_is_distant {
                return None;
            }

            // Detached dynamic voxels and fragments are no longer spatially contained by
            // their source chunk. Keep that chunk resident while one of its actors is still
            // near the camera, otherwise unloading would make nearby debris disappear.
            let has_nearby_dynamic_body = bodies.iter().any(
                |(_, membership, transform, _, _)| {
                    membership.0 == *id
                        && camera_position.distance(transform.translation) <= unload_distance
                },
            );
            let has_nearby_fragment = fragments.iter().any(
                |(_, _, membership, transform, _, _)| {
                    membership.0 == *id
                        && camera_position.distance(transform.translation) <= unload_distance
                },
            );

            (!has_nearby_dynamic_body && !has_nearby_fragment).then(|| id.clone())
        })
        .collect();

    for chunk_id in loaded_to_unload {
        let mut owned_entities = HashSet::<Entity>::new();
        for (entity, membership) in &chunk_entities {
            if membership.0 == chunk_id {
                owned_entities.insert(entity);
            }
        }

        for (id, membership, transform, linear_velocity, angular_velocity) in &bodies {
            if membership.0 != chunk_id {
                continue;
            }
            world_state.bodies.insert(
                id.0.clone(),
                PersistedBodyState {
                    translation_m: transform.translation.to_array(),
                    rotation_xyzw: transform.rotation.to_array(),
                    linear_velocity_m_s: linear_velocity.0.to_array(),
                    angular_velocity_rad_s: angular_velocity.0.to_array(),
                },
            );
        }

        for (id, fragment, membership, transform, linear_velocity, angular_velocity) in &fragments {
            if membership.0 != chunk_id {
                continue;
            }
            world_state.fragments.insert(
                id.0.clone(),
                PersistedFragmentState {
                    chunk_id: fragment.chunk_id.clone(),
                    definition_id: fragment.definition_id.clone(),
                    half_extents_m: fragment.half_extents_m,
                    translation_m: transform.translation.to_array(),
                    rotation_xyzw: transform.rotation.to_array(),
                    linear_velocity_m_s: linear_velocity.0.to_array(),
                    angular_velocity_rad_s: angular_velocity.0.to_array(),
                },
            );
        }

        // A bond is resident only while both endpoint bodies are resident.
        for (joint_entity, bond) in &bonds {
            if owned_entities.contains(&bond.body1) || owned_entities.contains(&bond.body2) {
                commands.entity(joint_entity).despawn();
                state.active_bond_ids.remove(&bond.id);
            }
        }
        for entity in owned_entities {
            commands.entity(entity).despawn();
        }

        state
            .voxel_bodies
            .retain(|_, body| body.chunk_id != chunk_id);
        state.loaded.remove(&chunk_id);
        info!("Unloaded chunk '{chunk_id}' after preserving its dynamic state");
    }

    // Discard completed reads that became irrelevant while waiting in the spawn queue.
    // This prevents stale ready items from consuming the per-frame spawn budget.
    state.ready.retain(|(source, _)| {
        distance_to_chunk_aabb(camera_position, source) <= chunk_radius_m(source, load_limit)
    });

    // Prioritize nearest indexed chunks and keep the number of outstanding I/O tasks bounded.
    let mut candidates: Vec<_> = config
        .chunks
        .iter()
        .filter(|source| {
            !state.loaded.contains(&source.id)
                && !state.failed.contains(&source.id)
                && !state.pending.contains_key(&source.id)
                && !state.ready.iter().any(|(queued, _)| queued.id == source.id)
                && distance_to_chunk_aabb(camera_position, source)
                    <= chunk_radius_m(source, load_limit)
        })
        .cloned()
        .collect();
    candidates.sort_by(|left, right| {
        distance_to_chunk_aabb(camera_position, left)
            .total_cmp(&distance_to_chunk_aabb(camera_position, right))
    });

    let available = config
        .engine
        .streaming
        .max_parallel_chunk_loads
        .saturating_sub(state.pending.len() as u32) as usize;
    for source in candidates.into_iter().take(available) {
        let path = source.path.clone();
        let id = source.id.clone();
        let task = AsyncComputeTaskPool::get().spawn(async move {
            let contents = fs::read_to_string(&path)
                .map_err(|error| format!("{}: {error}", path.display()))?;
            serde_json::from_str::<ChunkFile>(&contents)
                .map_err(|error| format!("{}: {error}", path.display()))
        });
        state.pending.insert(id, PendingChunk { source, task });
    }

    // Spawn only a bounded number of ready chunks per rendered frame. A completed payload
    // that moved outside the load radius while it was being read is discarded.
    for _ in 0..config.engine.streaming.max_chunk_spawns_per_frame {
        let Some((source, chunk)) = state.ready.pop_front() else {
            break;
        };
        if distance_to_chunk_aabb(camera_position, &source)
            > chunk_radius_m(&source, load_limit)
        {
            continue;
        }
        let spawn_result = {
            let streaming_state = state.bypass_change_detection();
            let (voxel_bodies, active_bond_ids) = (
                &mut streaming_state.voxel_bodies,
                &mut streaming_state.active_bond_ids,
            );
            spawn::spawn_chunk(
                &mut commands,
                &chunk,
                &config,
                &mut world_state,
                &journal,
                voxel_bodies,
                active_bond_ids,
                &asset_server,
                &mut meshes,
                &mut materials,
            )
        };
        match spawn_result {
            Ok(body_count) => {
                state.loaded.insert(source.id.clone());
                info!(
                    "Streamed chunk '{}' from region '{}' at {:?} with {} individual interactive bodies",
                    source.id, source.region_id, source.region_coordinate, body_count
                );
            }
            Err(error) => {
                state.failed.insert(source.id.clone());
                error!("Failed to spawn chunk '{}': {error}", source.id);
            },
        }
    }
}

fn chunk_bounds(source: &ChunkSource) -> (Vec3, Vec3) {
    let size = Vec3::new(
        source.dimensions[0] as f32 * source.voxel_size_m,
        source.dimensions[1] as f32 * source.voxel_size_m,
        source.dimensions[2] as f32 * source.voxel_size_m,
    );
    let min = Vec3::new(
        source.coordinate[0] as f32 * size.x,
        source.coordinate[1] as f32 * size.y,
        source.coordinate[2] as f32 * size.z,
    );
    (min, min + size)
}

fn distance_to_chunk_aabb(point: Vec3, source: &ChunkSource) -> f32 {
    let (min, max) = chunk_bounds(source);
    let closest = point.clamp(min, max);
    point.distance(closest)
}

fn chunk_radius_m(source: &ChunkSource, radius_chunks: f32) -> f32 {
    let (min, max) = chunk_bounds(source);
    (max - min).max_element() * radius_chunks
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::{chunk_radius_m, distance_to_chunk_aabb};
    use crate::config::ChunkSource;
    use bevy::prelude::Vec3;

    fn source() -> ChunkSource {
        ChunkSource {
            id: "chunk_1_-1_0".to_owned(),
            region_id: "overworld".to_owned(),
            region_coordinate: [0, 0],
            coordinate: [1, -1, 0],
            dimensions: [16, 16, 16],
            voxel_size_m: 1.0,
            path: PathBuf::from("regions/chunks/chunk_1_-1_0.json"),
        }
    }

    #[test]
    fn chunk_bounds_and_distance_work_for_negative_coordinates() {
        let chunk = source();
        assert_eq!(
            distance_to_chunk_aabb(Vec3::new(20.0, -4.0, 8.0), &chunk),
            0.0
        );
        assert!((distance_to_chunk_aabb(Vec3::new(35.0, -4.0, 8.0), &chunk) - 3.0).abs() < 1.0e-5);
        assert!((distance_to_chunk_aabb(Vec3::new(20.0, -20.0, 8.0), &chunk) - 4.0).abs() < 1.0e-5);
    }

    #[test]
    fn streaming_radius_scales_with_the_authored_chunk_bounds() {
        let chunk = source();
        assert_eq!(chunk_radius_m(&chunk, 2.0), 32.0);
        assert_eq!(chunk_radius_m(&chunk, 3.0), 48.0);
    }
}

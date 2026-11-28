use std::{
    collections::{HashMap, HashSet, VecDeque},
    fs,
};

use bevy::{
    prelude::*,
    tasks::{AsyncComputeTaskPool, Task, block_on, futures_lite::future::poll_once},
};

use super::{
    WorldOrigin,
    spawn::{self, FarChunkLodMesh},
    streaming::ChunkStreamingState,
};
use crate::{
    camera::FreeFlyCamera,
    config::{
        AlphaMode, ChunkFile, ChunkSource, RuntimeConfig, VisualDefinition, validate_chunk_file,
    },
    engine::AppMode,
    physics::DestructionJournal,
};

#[derive(Component)]
pub(super) struct FarChunkLod {
    chunk_id: String,
    half_extents: Vec3,
}

struct PendingFarChunk {
    source: ChunkSource,
    task: Task<Result<ChunkFile, String>>,
}

#[derive(Clone, Copy, PartialEq)]
struct FarChunkLodSettings {
    unload_radius_chunks: u32,
    visibility_distance_chunks: u32,
    max_far_lod_chunks: u32,
    lod_quality: u8,
    geometry_errors_m: [u32; 3],
}

impl From<&RuntimeConfig> for FarChunkLodSettings {
    fn from(config: &RuntimeConfig) -> Self {
        let streaming = &config.engine.streaming;
        Self {
            unload_radius_chunks: streaming.unload_radius_chunks,
            visibility_distance_chunks: streaming.lod_visibility_distance_chunks,
            max_far_lod_chunks: streaming.max_far_lod_chunks,
            lod_quality: streaming.lod_quality,
            geometry_errors_m: streaming.lod_geometry_errors_m.map(f32::to_bits),
        }
    }
}

/// In-memory far-chunk cache. Cached meshes intentionally outlive resident physics chunks.
#[derive(Resource, Default)]
pub(super) struct FarChunkLodState {
    pending: HashMap<String, PendingFarChunk>,
    ready: VecDeque<(ChunkSource, ChunkFile)>,
    desired_sources: Vec<ChunkSource>,
    cached: HashSet<String>,
    failed: HashSet<String>,
    last_camera_cell: Option<[i64; 3]>,
    settings: Option<FarChunkLodSettings>,
}

type FarChunkLodQuery<'w, 's> = Query<
    'w,
    's,
    (Entity, &'static FarChunkLod, &'static Transform, &'static mut Visibility),
>;

#[derive(SystemParam)]
pub(super) struct FarChunkLodSystem<'w, 's> {
    mode: Res<'w, State<AppMode>>,
    config: Res<'w, RuntimeConfig>,
    world_origin: Res<'w, WorldOrigin>,
    camera: Query<'w, 's, &'static Transform, With<FreeFlyCamera>>,
    journal: Res<'w, DestructionJournal>,
    streaming: Res<'w, ChunkStreamingState>,
}

pub(super) fn update_far_chunk_lod(
    context: FarChunkLodSystem,
    mut state: ResMut<FarChunkLodState>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut far_entities: FarChunkLodQuery<'_, '_>,
) {
    if *context.mode.get() != AppMode::Simulation {
        for (_, _, _, mut visibility) in &mut far_entities {
            visibility.set_if_neq(Visibility::Hidden);
        }
        return;
    }

    if !context.config.engine.streaming.lod_enabled
        || context.config.engine.streaming.max_far_lod_chunks == 0
        || context.config.chunks.is_empty()
    {
        state.pending.clear();
        state.ready.clear();
        state.desired_sources.clear();
        state.cached.clear();
        state.failed.clear();
        state.last_camera_cell = None;
        state.settings = None;
        for (entity, _, _, _) in &far_entities {
            commands.entity(entity).despawn();
        }
        return;
    }

    let Some(camera_transform) = context.camera.iter().next() else {
        for (_, _, _, mut visibility) in &mut far_entities {
            visibility.set_if_neq(Visibility::Hidden);
        }
        return;
    };
    let camera_position = camera_transform.translation;
    let settings = FarChunkLodSettings::from(&context.config);
    let reference_source = &context.config.chunks[0];
    let reference_extent = chunk_extent(reference_source);
    let camera_cell = [
        (f64::from(camera_position.x) + context.world_origin.0.x).div_euclid(reference_extent.x)
            .floor() as i64,
        (f64::from(camera_position.y) + context.world_origin.0.y).div_euclid(reference_extent.y)
            .floor() as i64,
        (f64::from(camera_position.z) + context.world_origin.0.z).div_euclid(reference_extent.z)
            .floor() as i64,
    ];
    let settings_changed = state.settings != Some(settings) || context.config.is_changed();
    let camera_cell_changed = state.last_camera_cell != Some(camera_cell);

    if settings_changed || camera_cell_changed {
        if settings_changed {
            state.pending.clear();
            state.ready.clear();
            state.cached.clear();
            state.failed.clear();
            for (entity, _, _, _) in &far_entities {
                commands.entity(entity).despawn();
            }
        }

        let mut candidates = context
            .config
            .chunks
            .iter()
            .filter(|source| !source.generated)
            .filter_map(|source| {
                let distance = distance_to_chunk_aabb(
                    camera_position,
                    source,
                    context.world_origin.0,
                );
                let full_detail_limit = chunk_radius_m(source, settings.unload_radius_chunks);
                let far_visibility_limit =
                    chunk_radius_m(source, settings.visibility_distance_chunks);
                if distance > full_detail_limit && distance <= far_visibility_limit {
                    Some((distance, source.clone()))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        candidates.sort_by(|left, right| left.0.total_cmp(&right.0));
        state.desired_sources = candidates
            .into_iter()
            .take(settings.max_far_lod_chunks as usize)
            .map(|(_, source)| source)
            .collect();
        state.last_camera_cell = Some(camera_cell);
        state.settings = Some(settings);

        let desired_ids = state
            .desired_sources
            .iter()
            .map(|source| source.id.clone())
            .collect::<HashSet<_>>();
        state.pending.retain(|id, _| desired_ids.contains(id));
        state.ready.retain(|(source, _)| desired_ids.contains(&source.id));
        state.failed.retain(|id| desired_ids.contains(id));

        let evicted = state
            .cached
            .iter()
            .filter(|id| !desired_ids.contains(*id))
            .cloned()
            .collect::<Vec<_>>();
        for id in &evicted {
            state.cached.remove(id);
        }
        if !evicted.is_empty() {
            for (entity, far, _, _) in &far_entities {
                if evicted.iter().any(|id| id == &far.chunk_id) {
                    commands.entity(entity).despawn();
                }
            }
        }
    }

    // Read chunk payloads on worker threads, sharing the same parallel-load budget as
    // full-detail streaming. The payload is validated before it can enter the mesh queue.
    let total_pending = context
        .streaming
        .pending_count()
        .saturating_add(state.pending.len())
        .min(u32::MAX as usize) as u32;
    let available_tasks = context
        .config
        .engine
        .streaming
        .max_parallel_chunk_loads
        .saturating_sub(total_pending) as usize;
    let desired_to_load = state
        .desired_sources
        .iter()
        .filter(|source| {
            !state.cached.contains(&source.id)
                && !state.failed.contains(&source.id)
                && !state.pending.contains_key(&source.id)
                && !state.ready.iter().any(|(ready, _)| ready.id == source.id)
        })
        .take(available_tasks)
        .cloned()
        .collect::<Vec<_>>();

    for source in desired_to_load {
        let path = source.path.clone();
        let source_for_task = source.clone();
        let components = context.config.components.clone();
        let task = AsyncComputeTaskPool::get().spawn(async move {
            let contents = fs::read_to_string(&path)
                .map_err(|error| format!("{}: {error}", path.display()))?;
            let chunk = serde_json::from_str::<ChunkFile>(&contents)
                .map_err(|error| format!("{}: {error}", path.display()))?;
            validate_chunk_file(&chunk, &source_for_task, &components)
                .map_err(|error| format!("{}: {error:#}", source_for_task.id))?;
            Ok(chunk)
        });
        state.pending.insert(
            source.id.clone(),
            PendingFarChunk {
                source,
                task,
            },
        );
    }

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
        if !state
            .desired_sources
            .iter()
            .any(|source| source.id == id)
        {
            continue;
        }
        match result {
            Ok(chunk) => state.ready.push_back((pending.source, chunk)),
            Err(error) => {
                state.failed.insert(id.clone());
                warn!("Far-LOD cache rejected chunk '{id}': {error}");
            }
        }
    }

    // Mesh generation and ECS insertion are budgeted separately from I/O. Far proxies
    // contain render components only and are not assigned ChunkMembership or physics.
    let build_budget = context.config.engine.streaming.max_chunk_spawns_per_frame as usize;
    for _ in 0..build_budget {
        let Some((source, chunk)) = state.ready.pop_front() else {
            break;
        };
        if !state
            .desired_sources
            .iter()
            .any(|desired| desired.id == source.id)
        {
            continue;
        }

        let quality_error =
            context.config.engine.streaming.lod_geometry_errors_m[settings.lod_quality as usize];
        let max_error_m = quality_error * 4.0;
        match spawn::build_far_chunk_lod_meshes(
            &chunk,
            &context.config.components,
            max_error_m,
        ) {
            Ok(far_meshes) => {
                spawn_far_chunk_meshes(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &source,
                    context.world_origin.0,
                    far_meshes,
                );
                // Cache empty results as well: chunks without static opaque geometry should
                // not be read and processed again while they remain in the selected window.
                state.cached.insert(source.id);
            }
            Err(error) => {
                state.failed.insert(source.id.clone());
                warn!("Could not build far-LOD geometry for '{}': {error}", source.id);
            }
        }
    }

    let damaged_chunks = context
        .journal
        .fractured
        .iter()
        .filter_map(|id| crate::world::parse_voxel_instance_id(id).map(|(chunk_id, _)| chunk_id))
        .collect::<HashSet<_>>();
    for (_, far, transform, mut visibility) in &mut far_entities {
        let min = transform.translation;
        let max = min + far.half_extents * 2.0;
        let distance = distance_to_aabb(camera_position, min, max);
        let chunk_is_loaded = context.streaming.is_loaded(&far.chunk_id);
        let damaged = damaged_chunks.contains(far.chunk_id.as_str());
        let outer_radius = far.half_extents.max_element() * 2.0
            * settings.visibility_distance_chunks as f32;
        let inner_radius =
            far.half_extents.max_element() * 2.0 * settings.unload_radius_chunks as f32;
        visibility.set_if_neq(if far_chunk_lod_is_visible(
            distance,
            inner_radius,
            outer_radius,
            chunk_is_loaded,
            damaged,
        ) {
            Visibility::Visible
        } else {
            Visibility::Hidden
        });
    }
}

fn spawn_far_chunk_meshes(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    source: &ChunkSource,
    world_origin: bevy::math::DVec3,
    far_meshes: Vec<FarChunkLodMesh>,
) {
    let dimensions = source.dimensions.map(f64::from);
    let voxel_size = f64::from(source.voxel_size_m);
    let chunk_origin = bevy::math::DVec3::new(
        f64::from(source.coordinate[0]) * dimensions[0] * voxel_size,
        f64::from(source.coordinate[1]) * dimensions[1] * voxel_size,
        f64::from(source.coordinate[2]) * dimensions[2] * voxel_size,
    ) - world_origin;
    let local_origin = chunk_origin.as_vec3();
    let half_extents = Vec3::from_array(
        source
            .dimensions
            .map(|dimension| dimension as f32 * source.voxel_size_m * 0.5),
    );

    for far_mesh in far_meshes {
        commands.spawn((
            Name::new(format!(
                "far-chunk:{}:{}",
                source.id, far_mesh.definition_id
            )),
            FarChunkLod {
                chunk_id: source.id.clone(),
                half_extents,
            },
            Mesh3d(meshes.add(far_mesh.mesh)),
            MeshMaterial3d(materials.add(standard_material_from_visual(&far_mesh.visual))),
            Transform::from_translation(local_origin),
            Visibility::Hidden,
        ));
    }
}

fn standard_material_from_visual(visual: &VisualDefinition) -> StandardMaterial {
    StandardMaterial {
        base_color: Color::srgba(
            visual.color_srgba[0],
            visual.color_srgba[1],
            visual.color_srgba[2],
            visual.color_srgba[3],
        ),
        perceptual_roughness: visual.perceptual_roughness,
        metallic: visual.metallic,
        alpha_mode: match visual.alpha_mode {
            AlphaMode::Opaque => AlphaMode::Opaque,
            AlphaMode::Blend => AlphaMode::Blend,
        },
        ..default()
    }
}

fn chunk_extent(source: &ChunkSource) -> bevy::math::DVec3 {
    bevy::math::DVec3::new(
        source.dimensions[0] as f64 * source.voxel_size_m as f64,
        source.dimensions[1] as f64 * source.voxel_size_m as f64,
        source.dimensions[2] as f64 * source.voxel_size_m as f64,
    )
}

fn chunk_radius_m(source: &ChunkSource, radius_chunks: u32) -> f32 {
    source.dimensions.iter().copied().max().unwrap_or(0) as f32
        * source.voxel_size_m
        * radius_chunks as f32
}

fn distance_to_chunk_aabb(
    point: Vec3,
    source: &ChunkSource,
    world_origin: bevy::math::DVec3,
) -> f32 {
    let extent = chunk_extent(source);
    let absolute_min = bevy::math::DVec3::new(
        source.coordinate[0] as f64 * extent.x,
        source.coordinate[1] as f64 * extent.y,
        source.coordinate[2] as f64 * extent.z,
    );
    let min = (absolute_min - world_origin).as_vec3();
    distance_to_aabb(point, min, min + extent.as_vec3())
}

fn distance_to_aabb(point: Vec3, min: Vec3, max: Vec3) -> f32 {
    point.distance(point.clamp(min, max))
}

fn far_chunk_lod_is_visible(
    distance_m: f32,
    inner_radius_m: f32,
    outer_radius_m: f32,
    chunk_is_loaded: bool,
    chunk_is_damaged: bool,
) -> bool {
    !chunk_is_loaded
        && !chunk_is_damaged
        && distance_m > inner_radius_m
        && distance_m <= outer_radius_m
}

#[cfg(test)]
mod tests {
    use super::far_chunk_lod_is_visible;

    #[test]
    fn far_chunk_proxy_only_fills_the_gap_between_resident_and_outer_radii() {
        assert!(!far_chunk_lod_is_visible(30.0, 30.0, 5120.0, false, false));
        assert!(far_chunk_lod_is_visible(30.1, 30.0, 5120.0, false, false));
        assert!(far_chunk_lod_is_visible(5120.0, 30.0, 5120.0, false, false));
        assert!(!far_chunk_lod_is_visible(5120.1, 30.0, 5120.0, false, false));
    }

    #[test]
    fn resident_or_damaged_chunks_never_show_the_far_snapshot() {
        assert!(!far_chunk_lod_is_visible(500.0, 30.0, 5120.0, true, false));
        assert!(!far_chunk_lod_is_visible(500.0, 30.0, 5120.0, false, true));
    }
}

use std::collections::{BTreeMap, HashMap, HashSet};

use avian3d::prelude::{
    AngularVelocity, CcdFilter, Collider, ColliderDensity, CollisionEventsEnabled, Friction,
    LinearVelocity, Restitution, RigidBody, SleepThreshold, SweepMode, SweptCcd,
};
use bevy::{gltf::prelude::GltfAssetLabel, math::DVec3, prelude::*};

use crate::{
    config::{
        AlphaMode, BodyType, BondStrengthDefinition, CcdFilterDefinition, ChunkFile, ColliderShape,
        ComponentDefinition, ObjectChunkSource, RuntimeConfig, SweepModeDefinition,
        VisualDefinition, object_chunk_coordinate,
    },
    persistence::{
        PersistRuntimeState, PersistedBodyState, PersistedFragment, PersistedFragmentState,
        PersistentObjectId, WorldStateFile,
    },
    physics::{
        DestructibleBody, DestructibleBodySpec, RollingResistance, VoxelBondSpec, spawn_voxel_bond,
    },
    world::{ChunkMembership, greedy_mesher::VoxelGrid, lod::ChunkLodVisual, voxel_instance_id},
};

fn object_lod_enabled(body_type: BodyType, lod_enabled: bool) -> bool {
    lod_enabled && matches!(body_type, BodyType::Static)
}

fn static_cell_render_policy(
    body_type: BodyType,
    intact: bool,
    destructible: bool,
    bonded: bool,
    greedy_meshing_enabled: bool,
) -> (bool, bool) {
    let lod_renderable = matches!(body_type, BodyType::Static) && intact;
    let greedy_mergeable = lod_renderable && greedy_meshing_enabled && !destructible && !bonded;
    (lod_renderable, greedy_mergeable)
}

#[derive(Clone)]
pub(crate) struct SpawnedVoxelBody {
    pub(crate) id: String,
    pub(crate) entity: Entity,
    pub(crate) chunk_id: String,
    pub(crate) rotation: Quat,
    pub(crate) half_extents: Vec3,
    pub(crate) body_type: BodyType,
    pub(crate) bond_strength: Option<BondStrengthDefinition>,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn spawn_object_chunk(
    commands: &mut Commands,
    source: &ObjectChunkSource,
    config: &RuntimeConfig,
    world_state: &WorldStateFile,
    world_origin: DVec3,
    asset_server: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let mut mesh_cache = HashMap::<MeshKey, Handle<Mesh>>::new();
    let mut material_cache = HashMap::<VisualMaterialKey, Handle<StandardMaterial>>::new();
    let chunk_size_m = config.engine.streaming.object_chunk_size_m;
    let chunk_extent_m = chunk_size_m;
    let chunk_center = (DVec3::from_array(
        source
            .coordinate
            .map(|axis| (axis as f64 + 0.5) * chunk_size_m as f64),
    ) - world_origin)
        .as_vec3();
    let lod_settings = &config.engine.streaming;

    for instance in &source.objects {
        if world_state.fractured.contains(&instance.id) {
            continue;
        }
        let definition = config
            .components
            .definitions
            .get(&instance.definition)
            .expect("component references are validated before app startup");
        let physical_material = config
            .materials
            .materials
            .get(&definition.material)
            .expect("material references are validated before app startup");
        let saved = if matches!(definition.body_type, BodyType::Dynamic) {
            world_state.bodies.get(&instance.id)
        } else {
            None
        };
        let transform = Transform {
            translation: (DVec3::from_array(instance.position_m.map(f64::from)) - world_origin)
                .as_vec3(),
            rotation: rotation_from_degrees(instance.rotation_degrees_xyz),
            ..default()
        };
        let entity = spawn_body(
            commands,
            &instance.id,
            &instance.definition,
            definition,
            physical_material,
            transform,
            saved,
            config,
            asset_server,
            meshes,
            materials,
            &mut mesh_cache,
            &mut material_cache,
        );
        let mut object_entity = commands.entity(entity);
        object_entity.insert(ChunkMembership(source.id.clone()));
        if object_lod_enabled(definition.body_type, lod_settings.lod_enabled) {
            object_entity.insert(ChunkLodVisual {
                chunk_id: source.id.clone(),
                level: 0,
                max_level: 0,
                center: chunk_center,
                half_extents: Vec3::splat(chunk_extent_m * 0.5),
                near_distance_m: lod_settings.lod_near_distance_chunks * chunk_extent_m,
                mid_distance_m: lod_settings.lod_mid_distance_chunks * chunk_extent_m,
                far_distance_m: lod_settings.lod_far_distance_chunks * chunk_extent_m,
                unload_distance_m: lod_settings.unload_radius_chunks as f32 * chunk_extent_m,
                fallback_on_damage: false,
            });
        }
    }

    // Older saves did not record an object-chunk ID for fragments created from authored
    // objects. Rehome those records by their last world position as they enter the new index.
    let mut restored_fragment_count = 0_usize;
    for (instance_id, state) in &world_state.fragments {
        let belongs_to_chunk = state.chunk_id.as_deref() == Some(source.id.as_str())
            || (state.chunk_id.is_none()
                && object_chunk_coordinate(state.translation_m, chunk_size_m)
                    .is_ok_and(|coordinate| coordinate == source.coordinate));
        if !belongs_to_chunk {
            continue;
        }
        let Some(definition) = config.components.definitions.get(&state.definition_id) else {
            warn!(
                "Skipping fragment '{}' because component definition '{}' is unavailable",
                instance_id, state.definition_id
            );
            continue;
        };
        let Some(physical_material) = config.materials.materials.get(&definition.material) else {
            warn!(
                "Skipping fragment '{}' because material '{}' is unavailable",
                instance_id, definition.material
            );
            continue;
        };
        let mut restored_state = state.clone();
        restored_state.chunk_id = Some(source.id.clone());
        spawn_saved_fragment(
            commands,
            instance_id,
            &restored_state,
            definition,
            physical_material,
            config,
            asset_server,
            meshes,
            materials,
            &mut mesh_cache,
            &mut material_cache,
        );
        restored_fragment_count += 1;
    }

    info!(
        "Streamed authored-object chunk '{}' with {} objects and {} restored fragments",
        source.id,
        source.objects.len(),
        restored_fragment_count
    );
}

/// Spawns one validated chunk. Static non-fracturable cells use greedy render meshes and
/// material-homogeneous compound colliders. Interactive cells remain independent rigid bodies.
#[allow(clippy::too_many_arguments)]
pub(super) fn spawn_chunk(
    commands: &mut Commands,
    chunk: &ChunkFile,
    config: &RuntimeConfig,
    world_origin: DVec3,
    world_state: &mut WorldStateFile,
    journal: &crate::physics::DestructionJournal,
    voxel_bodies: &mut HashMap<[i32; 3], SpawnedVoxelBody>,
    active_bond_ids: &mut HashSet<String>,
    asset_server: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) -> Result<usize, String> {
    let dimensions = [
        chunk.dimensions[0] as usize,
        chunk.dimensions[1] as usize,
        chunk.dimensions[2] as usize,
    ];
    let volume = dimensions[0] * dimensions[1] * dimensions[2];
    let mut cells = vec![0_u8; volume];
    let index = |x: usize, y: usize, z: usize| (y * dimensions[2] + z) * dimensions[0] + x;

    for layer in &chunk.layers {
        for (z, row) in layer.rows.iter().enumerate() {
            for (x, character) in row.chars().enumerate() {
                cells[index(x, layer.y as usize, z)] = character.to_digit(16).ok_or_else(|| {
                    format!("chunk '{}' contains a non-hex palette cell", chunk.id)
                })? as u8;
            }
        }
    }

    let grid = VoxelGrid::from_cells(dimensions, chunk.voxel_size_m, cells)
        .map_err(|error| format!("chunk '{}' has invalid voxel data: {error}", chunk.id))?;

    let mut mergeable = vec![false; volume];
    let mut lod_renderable = vec![false; volume];
    let mut definitions_by_palette = BTreeMap::new();
    for (palette_index, definition_id) in chunk.palette.iter().enumerate().skip(1) {
        let definition = config
            .components
            .definitions
            .get(definition_id)
            .ok_or_else(|| {
                format!(
                    "chunk '{}' references missing definition '{definition_id}'",
                    chunk.id
                )
            })?;
        definitions_by_palette.insert(
            palette_index as u8,
            (definition_id.clone(), definition.clone()),
        );
    }

    for y in 0..dimensions[1] {
        for z in 0..dimensions[2] {
            for x in 0..dimensions[0] {
                let palette = grid.cell(x, y, z);
                if palette == 0 {
                    continue;
                }
                let Some((_, definition)) = definitions_by_palette.get(&palette) else {
                    return Err(format!(
                        "chunk '{}' references palette index {palette} outside its palette",
                        chunk.id
                    ));
                };
                let instance_id = voxel_instance_id(&chunk.id, x as u32, y as u32, z as u32);
                let intact = !journal.fractured.contains(&instance_id);
                let (is_lod_renderable, is_greedy_mergeable) = static_cell_render_policy(
                    definition.body_type,
                    intact,
                    definition.destructibility.is_some(),
                    definition.bond_strength.is_some(),
                    config.engine.streaming.greedy_meshing_enabled,
                );
                lod_renderable[index(x, y, z)] = is_lod_renderable
                    && definition.visual.model.is_none()
                    && matches!(definition.visual.alpha_mode, AlphaMode::Opaque);
                mergeable[index(x, y, z)] = is_greedy_mergeable
                    && definition.visual.model.is_none()
                    && matches!(definition.visual.alpha_mode, AlphaMode::Opaque);
            }
        }
    }

    let chunk_origin = (DVec3::new(
        chunk.coordinate[0] as f64 * dimensions[0] as f64 * chunk.voxel_size_m as f64,
        chunk.coordinate[1] as f64 * dimensions[1] as f64 * chunk.voxel_size_m as f64,
        chunk.coordinate[2] as f64 * dimensions[2] as f64 * chunk.voxel_size_m as f64,
    ) - world_origin)
        .as_vec3();

    let lod_settings = &config.engine.streaming;
    let chunk_extent_m = dimensions.iter().copied().max().unwrap_or(0) as f32 * chunk.voxel_size_m;
    let lod_center = chunk_origin
        + Vec3::from_array(dimensions.map(|dimension| dimension as f32 * chunk.voxel_size_m * 0.5));
    let lod_near_distance_m = lod_settings.lod_near_distance_chunks * chunk_extent_m;
    let lod_mid_distance_m = lod_settings.lod_mid_distance_chunks * chunk_extent_m;
    let lod_far_distance_m = lod_settings.lod_far_distance_chunks * chunk_extent_m;
    let lod_unload_distance_m = lod_settings.unload_radius_chunks as f32 * chunk_extent_m;
    let max_lod_level = if dimensions.iter().all(|dimension| dimension % 8 == 0) {
        3
    } else if dimensions.iter().all(|dimension| dimension % 4 == 0) {
        2
    } else if dimensions.iter().all(|dimension| dimension % 2 == 0) {
        1
    } else {
        0
    };
    // Batch static cells by palette entry. This keeps independent physical material response
    // while dramatically reducing entities, draw calls, and collider shape count for terrain.
    if config.engine.streaming.greedy_meshing_enabled {
        let boxes = grid.greedy_boxes(&mergeable).map_err(|error| {
            format!(
                "chunk '{}' greedy collider generation failed: {error}",
                chunk.id
            )
        })?;
        let mut collider_shapes = BTreeMap::<u8, Vec<(Vec3, Quat, Collider)>>::new();
        for greedy_box in boxes {
            collider_shapes
                .entry(greedy_box.palette_index)
                .or_default()
                .push((
                    greedy_box.center_m,
                    Quat::IDENTITY,
                    Collider::cuboid(
                        greedy_box.full_size_m.x,
                        greedy_box.full_size_m.y,
                        greedy_box.full_size_m.z,
                    ),
                ));
        }

        let mut mesh_by_palette = grid.build_greedy_meshes(&mergeable).map_err(|error| {
            format!(
                "chunk '{}' greedy mesh generation failed: {error}",
                chunk.id
            )
        })?;
        for (palette, shapes) in collider_shapes {
            let Some((definition_id, definition)) = definitions_by_palette.get(&palette) else {
                continue;
            };
            let physical_material = config
                .materials
                .materials
                .get(&definition.material)
                .ok_or_else(|| {
                    format!(
                        "chunk '{}' definition '{}' has no physical material",
                        chunk.id, definition_id
                    )
                })?;
            let Some(mesh) = mesh_by_palette.remove(&palette) else {
                continue;
            };

            let mut static_components = commands.spawn((
                Name::new(format!("chunk:{}:static:{}", chunk.id, definition_id)),
                ChunkMembership(chunk.id.clone()),
                RigidBody::Static,
                Collider::compound(shapes),
                ColliderDensity(physical_material.density_kg_m3),
                Friction::new(physical_material.dynamic_friction)
                    .with_static_coefficient(physical_material.static_friction),
                Restitution::new(physical_material.restitution),
                CollisionEventsEnabled,
                Mesh3d(meshes.add(mesh)),
                MeshMaterial3d(materials.add(standard_material_from_visual(&definition.visual))),
                Transform::from_translation(chunk_origin),
                if lod_settings.lod_enabled {
                    Visibility::Hidden
                } else {
                    Visibility::Visible
                },
            ));
            if lod_settings.lod_enabled {
                static_components.insert(ChunkLodVisual {
                    chunk_id: chunk.id.clone(),
                    level: 0,
                    max_level: 0,
                    center: lod_center,
                    half_extents: Vec3::from_array(
                        dimensions.map(|dimension| dimension as f32 * chunk.voxel_size_m * 0.5),
                    ),
                    near_distance_m: lod_near_distance_m,
                    mid_distance_m: lod_mid_distance_m,
                    far_distance_m: lod_far_distance_m,
                    unload_distance_m: lod_unload_distance_m,
                    fallback_on_damage: true,
                });
            }
        }
    }

    // LOD0 is a full-resolution, render-only greedy surface over every intact static opaque
    // voxel. Destructible and bonded cells keep their independent physics bodies; their cube
    // renderers are hidden below while this merged surface is intact.
    if lod_settings.lod_enabled {
        let detailed_meshes = grid.build_greedy_meshes(&lod_renderable).map_err(|error| {
            format!("chunk '{}' LOD0 mesh generation failed: {error}", chunk.id)
        })?;
        for (palette, mesh) in detailed_meshes {
            let Some((definition_id, definition)) = definitions_by_palette.get(&palette) else {
                continue;
            };
            commands.spawn((
                Name::new(format!("chunk:{}:lod0:{}", chunk.id, definition_id)),
                ChunkMembership(chunk.id.clone()),
                Mesh3d(meshes.add(mesh)),
                MeshMaterial3d(materials.add(standard_material_from_visual(&definition.visual))),
                Transform::from_translation(chunk_origin),
                Visibility::Visible,
                ChunkLodVisual {
                    chunk_id: chunk.id.clone(),
                    level: 0,
                    max_level: max_lod_level,
                    center: lod_center,
                    half_extents: Vec3::from_array(
                        dimensions.map(|dimension| dimension as f32 * chunk.voxel_size_m * 0.5),
                    ),
                    near_distance_m: lod_near_distance_m,
                    mid_distance_m: lod_mid_distance_m,
                    far_distance_m: lod_far_distance_m,
                    unload_distance_m: lod_unload_distance_m,
                    fallback_on_damage: false,
                },
            ));
        }
    }

    // Progressive 2x, 4x and 8x visual proxies use the same intact-static source mask.
    if lod_settings.lod_enabled && dimensions.iter().all(|dimension| dimension % 2 == 0) {
        let coarse = grid
            .downsample_by_two(&lod_renderable)
            .map_err(|error| format!("chunk '{}' LOD downsampling failed: {error}", chunk.id))?;
        let coarse_renderable: Vec<bool> =
            (0..coarse.dimensions[0] * coarse.dimensions[1] * coarse.dimensions[2])
                .map(|linear| {
                    coarse.cell(
                        linear % coarse.dimensions[0],
                        (linear / coarse.dimensions[0]) / coarse.dimensions[2],
                        (linear / coarse.dimensions[0]) % coarse.dimensions[2],
                    ) != 0
                })
                .collect();
        for (palette, mesh) in coarse
            .build_greedy_meshes(&coarse_renderable)
            .map_err(|error| {
                format!(
                    "chunk '{}' coarse mesh generation failed: {error}",
                    chunk.id
                )
            })?
        {
            let Some((definition_id, definition)) = definitions_by_palette.get(&palette) else {
                continue;
            };
            commands.spawn((
                Name::new(format!("chunk:{}:lod1:{}", chunk.id, definition_id)),
                ChunkMembership(chunk.id.clone()),
                Mesh3d(meshes.add(mesh)),
                MeshMaterial3d(materials.add(standard_material_from_visual(&definition.visual))),
                Transform::from_translation(chunk_origin),
                Visibility::Hidden,
                ChunkLodVisual {
                    chunk_id: chunk.id.clone(),
                    level: 1,
                    max_level: max_lod_level,
                    center: lod_center,
                    half_extents: Vec3::from_array(
                        dimensions.map(|dimension| dimension as f32 * chunk.voxel_size_m * 0.5),
                    ),
                    near_distance_m: lod_near_distance_m,
                    mid_distance_m: lod_mid_distance_m,
                    far_distance_m: lod_far_distance_m,
                    unload_distance_m: lod_unload_distance_m,
                    fallback_on_damage: false,
                },
            ));
        }

        // A second factor-two pass provides a far-distance LOD when the authored chunk
        // dimensions support exact 4x aggregation on every axis.
        if lod_settings.lod_enabled && max_lod_level >= 2 {
            let far_grid = coarse
                .downsample_by_two(&coarse_renderable)
                .map_err(|error| {
                    format!("chunk '{}' far LOD downsampling failed: {error}", chunk.id)
                })?;
            let far_renderable: Vec<bool> =
                (0..far_grid.dimensions[0] * far_grid.dimensions[1] * far_grid.dimensions[2])
                    .map(|linear| {
                        far_grid.cell(
                            linear % far_grid.dimensions[0],
                            (linear / far_grid.dimensions[0]) / far_grid.dimensions[2],
                            (linear / far_grid.dimensions[0]) % far_grid.dimensions[2],
                        ) != 0
                    })
                    .collect();

            for (palette, mesh) in
                far_grid
                    .build_greedy_meshes(&far_renderable)
                    .map_err(|error| {
                        format!(
                            "chunk '{}' far LOD mesh generation failed: {error}",
                            chunk.id
                        )
                    })?
            {
                let Some((definition_id, definition)) = definitions_by_palette.get(&palette) else {
                    continue;
                };
                commands.spawn((
                    Name::new(format!("chunk:{}:lod2:{}", chunk.id, definition_id)),
                    ChunkMembership(chunk.id.clone()),
                    Mesh3d(meshes.add(mesh)),
                    MeshMaterial3d(
                        materials.add(standard_material_from_visual(&definition.visual)),
                    ),
                    Transform::from_translation(chunk_origin),
                    Visibility::Hidden,
                    ChunkLodVisual {
                        chunk_id: chunk.id.clone(),
                        level: 2,
                        max_level: max_lod_level,
                        center: lod_center,
                        half_extents: Vec3::from_array(
                            dimensions.map(|dimension| dimension as f32 * chunk.voxel_size_m * 0.5),
                        ),
                        near_distance_m: lod_near_distance_m,
                        mid_distance_m: lod_mid_distance_m,
                        far_distance_m: lod_far_distance_m,
                        unload_distance_m: lod_unload_distance_m,
                        fallback_on_damage: false,
                    },
                ));
            }

            if max_lod_level >= 3 {
                let ultra_far_grid =
                    far_grid
                        .downsample_by_two(&far_renderable)
                        .map_err(|error| {
                            format!(
                                "chunk '{}' ultra-far LOD downsampling failed: {error}",
                                chunk.id
                            )
                        })?;
                let ultra_far_renderable: Vec<bool> = (0..ultra_far_grid.dimensions[0]
                    * ultra_far_grid.dimensions[1]
                    * ultra_far_grid.dimensions[2])
                    .map(|linear| {
                        ultra_far_grid.cell(
                            linear % ultra_far_grid.dimensions[0],
                            (linear / ultra_far_grid.dimensions[0]) / ultra_far_grid.dimensions[2],
                            (linear / ultra_far_grid.dimensions[0]) % ultra_far_grid.dimensions[2],
                        ) != 0
                    })
                    .collect();

                for (palette, mesh) in ultra_far_grid
                    .build_greedy_meshes(&ultra_far_renderable)
                    .map_err(|error| {
                        format!(
                            "chunk '{}' ultra-far LOD mesh generation failed: {error}",
                            chunk.id
                        )
                    })?
                {
                    let Some((definition_id, definition)) = definitions_by_palette.get(&palette)
                    else {
                        continue;
                    };
                    commands.spawn((
                        Name::new(format!("chunk:{}:lod3:{}", chunk.id, definition_id)),
                        ChunkMembership(chunk.id.clone()),
                        Mesh3d(meshes.add(mesh)),
                        MeshMaterial3d(
                            materials.add(standard_material_from_visual(&definition.visual)),
                        ),
                        Transform::from_translation(chunk_origin),
                        Visibility::Hidden,
                        ChunkLodVisual {
                            chunk_id: chunk.id.clone(),
                            level: 3,
                            max_level: max_lod_level,
                            center: lod_center,
                            half_extents: Vec3::from_array(
                                dimensions
                                    .map(|dimension| dimension as f32 * chunk.voxel_size_m * 0.5),
                            ),
                            near_distance_m: lod_near_distance_m,
                            mid_distance_m: lod_mid_distance_m,
                            far_distance_m: lod_far_distance_m,
                            unload_distance_m: lod_unload_distance_m,
                            fallback_on_damage: false,
                        },
                    ));
                }
            }
        }
    }

    let mut newly_spawned = Vec::<[i32; 3]>::new();
    let mut visual_mesh_cache = HashMap::<MeshKey, Handle<Mesh>>::new();
    let mut visual_material_cache = HashMap::<VisualMaterialKey, Handle<StandardMaterial>>::new();

    // Non-static, destructible, bonded or imported-visual cells keep individual physical bodies.
    for y in 0..dimensions[1] {
        for z in 0..dimensions[2] {
            for x in 0..dimensions[0] {
                let palette = grid.cell(x, y, z);
                if palette == 0 || mergeable[index(x, y, z)] {
                    continue;
                }
                let (definition_id, authored_definition) =
                    definitions_by_palette.get(&palette).ok_or_else(|| {
                        format!(
                            "chunk '{}' references invalid palette index {palette}",
                            chunk.id
                        )
                    })?;
                let instance_id = voxel_instance_id(&chunk.id, x as u32, y as u32, z as u32);
                if journal.fractured.contains(&instance_id) {
                    continue;
                }

                let physical_material = config
                    .materials
                    .materials
                    .get(&authored_definition.material)
                    .ok_or_else(|| {
                        format!(
                            "component '{}' has unknown material '{}'",
                            definition_id, authored_definition.material
                        )
                    })?;
                let mut definition = authored_definition.clone();
                let ColliderShape::Cuboid { half_extents } = &mut definition.collider else {
                    return Err(format!(
                        "chunk '{}' palette definition '{}' is not a cuboid",
                        chunk.id, definition_id
                    ));
                };
                for extent in half_extents {
                    *extent *= chunk.voxel_size_m;
                }

                let saved = if matches!(definition.body_type, BodyType::Dynamic) {
                    world_state.bodies.get(&instance_id)
                } else {
                    None
                };
                let position = chunk_origin
                    + Vec3::new(
                        (x as f32 + 0.5) * chunk.voxel_size_m,
                        (y as f32 + 0.5) * chunk.voxel_size_m,
                        (z as f32 + 0.5) * chunk.voxel_size_m,
                    );
                let entity = spawn_body(
                    commands,
                    &instance_id,
                    definition_id,
                    &definition,
                    physical_material,
                    Transform::from_translation(position),
                    saved,
                    config,
                    asset_server,
                    meshes,
                    materials,
                    &mut visual_mesh_cache,
                    &mut visual_material_cache,
                );
                let mut chunk_entity = commands.entity(entity);
                chunk_entity.insert(ChunkMembership(chunk.id.clone()));
                if lod_settings.lod_enabled && lod_renderable[index(x, y, z)] {
                    // Keep exact per-cell rendering available for the damaged-chunk fallback,
                    // but avoid drawing it over the merged full-resolution LOD0 surface.
                    chunk_entity.insert(Visibility::Hidden);
                    chunk_entity.insert(ChunkLodVisual {
                        chunk_id: chunk.id.clone(),
                        level: 0,
                        max_level: 0,
                        center: lod_center,
                        half_extents: Vec3::from_array(
                            dimensions.map(|dimension| dimension as f32 * chunk.voxel_size_m * 0.5),
                        ),
                        near_distance_m: lod_near_distance_m,
                        mid_distance_m: lod_mid_distance_m,
                        far_distance_m: lod_far_distance_m,
                        unload_distance_m: lod_unload_distance_m,
                        fallback_on_damage: true,
                    });
                }

                let ColliderShape::Cuboid { half_extents } = &definition.collider else {
                    unreachable!("chunk palette definitions were checked above");
                };
                let coordinate = [
                    chunk.coordinate[0] * chunk.dimensions[0] as i32 + x as i32,
                    chunk.coordinate[1] * chunk.dimensions[1] as i32 + y as i32,
                    chunk.coordinate[2] * chunk.dimensions[2] as i32 + z as i32,
                ];
                voxel_bodies.insert(
                    coordinate,
                    SpawnedVoxelBody {
                        id: instance_id,
                        entity,
                        chunk_id: chunk.id.clone(),
                        rotation: saved
                            .map(|state| Quat::from_array(state.rotation_xyzw))
                            .unwrap_or(Quat::IDENTITY),
                        half_extents: Vec3::from_array(*half_extents),
                        body_type: definition.body_type,
                        bond_strength: definition.bond_strength.clone(),
                    },
                );
                newly_spawned.push(coordinate);
            }
        }
    }

    let newly_spawned_count = newly_spawned.len();

    // Reconnect new cells to any already-loaded neighbours. The stable pair ID prevents
    // duplicate joints when chunks load in a different order.
    let directions = [
        ([1, 0, 0], Vec3::X, 0_usize),
        ([-1, 0, 0], -Vec3::X, 0_usize),
        ([0, 1, 0], Vec3::Y, 1_usize),
        ([0, -1, 0], -Vec3::Y, 1_usize),
        ([0, 0, 1], Vec3::Z, 2_usize),
        ([0, 0, -1], -Vec3::Z, 2_usize),
    ];
    for coordinate in newly_spawned.iter().copied() {
        let Some(first) = voxel_bodies.get(&coordinate).cloned() else {
            continue;
        };
        for (offset, direction, axis) in directions {
            let neighbour_coordinate = [
                coordinate[0] + offset[0],
                coordinate[1] + offset[1],
                coordinate[2] + offset[2],
            ];
            let Some(second) = voxel_bodies.get(&neighbour_coordinate).cloned() else {
                continue;
            };
            if matches!(first.body_type, BodyType::Static)
                && matches!(second.body_type, BodyType::Static)
            {
                continue;
            }
            let Some(strength) =
                combine_bond_strength(first.bond_strength.as_ref(), second.bond_strength.as_ref())
            else {
                continue;
            };

            // Canonical endpoint order keeps the local bond frame independent of which chunk
            // happened to stream first. The neighbour scan itself visits both signs.
            let (body_a, body_b, direction_a) = if first.id < second.id {
                (&first, &second, direction)
            } else {
                (&second, &first, -direction)
            };
            let bond_id = format!("bond:{}|{}", body_a.id, body_b.id);
            if active_bond_ids.contains(&bond_id) || journal.broken_bonds.contains(&bond_id) {
                continue;
            }

            let local_axis1 = body_a.rotation.inverse() * direction_a;
            let local_axis2 = body_b.rotation.inverse() * -direction_a;
            let anchor1 = local_axis1 * body_a.half_extents[axis];
            let anchor2 = local_axis2 * body_b.half_extents[axis];
            let other_axes = match axis {
                0 => [1, 2],
                1 => [0, 2],
                _ => [0, 1],
            };
            let face_area = (body_a.half_extents[other_axes[0]] * 2.0)
                * (body_a.half_extents[other_axes[1]] * 2.0);
            let face_size = face_area.sqrt();
            let joint = spawn_voxel_bond(
                commands,
                VoxelBondSpec {
                    id: bond_id.clone(),
                    body1: body_a.entity,
                    body2: body_b.entity,
                    local_anchor1: anchor1,
                    local_anchor2: anchor2,
                    normal_local: local_axis1,
                    face_size_m: face_size,
                    strength,
                },
            );
            commands
                .entity(joint)
                .insert(ChunkMembership(chunk.id.clone()));
            active_bond_ids.insert(bond_id);
        }
    }

    // Restore detached fragments only when their owning chunk enters the active set.
    let restored_ids: Vec<_> = world_state
        .fragments
        .iter()
        .filter(|(_, state)| state.chunk_id.as_deref() == Some(chunk.id.as_str()))
        .map(|(id, _)| id.clone())
        .collect();
    for id in restored_ids {
        let state =
            world_state.fragments.get(&id).cloned().ok_or_else(|| {
                format!("fragment state '{id}' disappeared during chunk restoration")
            })?;
        let Some(definition) = config.components.definitions.get(&state.definition_id) else {
            warn!(
                "Preserving streamed fragment '{}' because component definition '{}' is unavailable",
                id, state.definition_id
            );
            continue;
        };
        let Some(physical_material) = config.materials.materials.get(&definition.material) else {
            warn!(
                "Preserving streamed fragment '{}' because material '{}' is unavailable",
                id, definition.material
            );
            continue;
        };
        spawn_saved_fragment(
            commands,
            &id,
            &state,
            definition,
            physical_material,
            config,
            asset_server,
            meshes,
            materials,
            &mut visual_mesh_cache,
            &mut visual_material_cache,
        );
        world_state.fragments.remove(&id);
    }

    Ok(newly_spawned_count)
}

#[allow(clippy::too_many_arguments)]
fn spawn_body(
    commands: &mut Commands,
    instance_id: &str,
    definition_id: &str,
    definition: &ComponentDefinition,
    physical_material: &crate::config::MaterialDefinition,
    authored_transform: Transform,
    saved: Option<&PersistedBodyState>,
    config: &RuntimeConfig,
    asset_server: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_cache: &mut HashMap<MeshKey, Handle<Mesh>>,
    material_cache: &mut HashMap<VisualMaterialKey, Handle<StandardMaterial>>,
) -> Entity {
    let (translation, rotation, linear_velocity, angular_velocity) = saved
        .map(|state| {
            (
                Vec3::from_array(state.translation_m),
                Quat::from_array(state.rotation_xyzw),
                Vec3::from_array(state.linear_velocity_m_s),
                Vec3::from_array(state.angular_velocity_rad_s),
            )
        })
        .unwrap_or((
            authored_transform.translation,
            authored_transform.rotation,
            Vec3::ZERO,
            Vec3::ZERO,
        ));

    let rigid_body = match definition.body_type {
        BodyType::Static => RigidBody::Static,
        BodyType::Dynamic => RigidBody::Dynamic,
    };
    let collider = collider_from_definition(&definition.collider);
    let friction = Friction::new(physical_material.dynamic_friction)
        .with_static_coefficient(physical_material.static_friction);

    let mut entity = commands.spawn((
        Name::new(instance_id.to_owned()),
        PersistentObjectId(instance_id.to_owned()),
        rigid_body,
        collider,
        ColliderDensity(physical_material.density_kg_m3),
        friction,
        Restitution::new(physical_material.restitution),
        CollisionEventsEnabled,
        LinearVelocity(linear_velocity),
        AngularVelocity(angular_velocity),
        Transform {
            translation,
            rotation,
            ..default()
        },
    ));

    if matches!(definition.body_type, BodyType::Dynamic) {
        entity.insert((
            PersistRuntimeState,
            SleepThreshold {
                linear: config.engine.physics.sleep_linear_threshold_m_s,
                angular: config.engine.physics.sleep_angular_threshold_rad_s,
            },
        ));
        if let Some(rolling) = RollingResistance::from_collider(
            &definition.collider,
            physical_material.density_kg_m3,
            physical_material.rolling_resistance_coefficient,
            1.0,
        ) {
            entity.insert(rolling);
        }
    }

    if let Some(ccd) = &definition.swept_ccd {
        let filter = match ccd.filter {
            CcdFilterDefinition::Default => CcdFilter::DEFAULT,
            CcdFilterDefinition::Dynamic => CcdFilter::DYNAMIC,
            CcdFilterDefinition::All => CcdFilter::ALL,
            CcdFilterDefinition::None => CcdFilter::NONE,
        };
        let mode = match ccd.mode {
            SweepModeDefinition::Linear => SweepMode::Linear,
            SweepModeDefinition::NonLinear => SweepMode::NonLinear,
        };
        entity.insert(SweptCcd {
            filter,
            mode,
            threshold: ccd.threshold,
        });
    }

    if let (Some(destructibility), ColliderShape::Cuboid { half_extents }) =
        (&definition.destructibility, &definition.collider)
    {
        entity.insert(DestructibleBody::from_spec(DestructibleBodySpec {
            instance_id,
            definition_id,
            destructibility,
            half_extents_m: Vec3::from_array(*half_extents),
            body_type: definition.body_type,
            density_kg_m3: physical_material.density_kg_m3,
            static_friction: physical_material.static_friction,
            dynamic_friction: physical_material.dynamic_friction,
            restitution: physical_material.restitution,
            rolling_resistance_coefficient: physical_material.rolling_resistance_coefficient,
            visual: &definition.visual,
        }));
    }

    let entity_id = entity.id();
    add_visual(
        &mut entity,
        definition,
        asset_server,
        meshes,
        materials,
        mesh_cache,
        material_cache,
    );
    entity_id
}

fn combine_bond_strength(
    first: Option<&BondStrengthDefinition>,
    second: Option<&BondStrengthDefinition>,
) -> Option<BondStrengthDefinition> {
    match (first, second) {
        (None, None) => None,
        (Some(strength), None) | (None, Some(strength)) => Some(strength.clone()),
        (Some(first), Some(second)) => Some(BondStrengthDefinition {
            elastic_modulus_pa: 2.0 * first.elastic_modulus_pa * second.elastic_modulus_pa
                / (first.elastic_modulus_pa + second.elastic_modulus_pa),
            tensile_strength_pa: first.tensile_strength_pa.min(second.tensile_strength_pa),
            compressive_strength_pa: first
                .compressive_strength_pa
                .min(second.compressive_strength_pa),
            shear_strength_pa: first.shear_strength_pa.min(second.shear_strength_pa),
            bending_strength_pa: first.bending_strength_pa.min(second.bending_strength_pa),
            torsional_strength_pa: first
                .torsional_strength_pa
                .min(second.torsional_strength_pa),
            damage_rate_per_s: first.damage_rate_per_s.max(second.damage_rate_per_s),
        }),
    }
}

#[allow(clippy::too_many_arguments)]
fn spawn_saved_fragment(
    commands: &mut Commands,
    instance_id: &str,
    state: &PersistedFragmentState,
    authored_definition: &ComponentDefinition,
    physical_material: &crate::config::MaterialDefinition,
    config: &RuntimeConfig,
    asset_server: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_cache: &mut HashMap<MeshKey, Handle<Mesh>>,
    material_cache: &mut HashMap<VisualMaterialKey, Handle<StandardMaterial>>,
) {
    let half_extents = Vec3::from_array(state.half_extents_m);
    let mut definition = authored_definition.clone();
    definition.collider = ColliderShape::Cuboid {
        half_extents: state.half_extents_m,
    };
    definition.destructibility = None;
    definition.bond_strength = None;

    let mut entity = commands.spawn((
        Name::new(instance_id.to_owned()),
        PersistentObjectId(instance_id.to_owned()),
        PersistedFragment {
            chunk_id: state.chunk_id.clone(),
            definition_id: state.definition_id.clone(),
            half_extents_m: state.half_extents_m,
        },
        RigidBody::Dynamic,
        Collider::cuboid(
            half_extents.x * 2.0,
            half_extents.y * 2.0,
            half_extents.z * 2.0,
        ),
        ColliderDensity(physical_material.density_kg_m3),
        Friction::new(physical_material.dynamic_friction)
            .with_static_coefficient(physical_material.static_friction),
        Restitution::new(physical_material.restitution),
        CollisionEventsEnabled,
        LinearVelocity(Vec3::from_array(state.linear_velocity_m_s)),
        AngularVelocity(Vec3::from_array(state.angular_velocity_rad_s)),
        SleepThreshold {
            linear: config.engine.physics.sleep_linear_threshold_m_s,
            angular: config.engine.physics.sleep_angular_threshold_rad_s,
        },
        Transform {
            translation: Vec3::from_array(state.translation_m),
            rotation: Quat::from_array(state.rotation_xyzw),
            ..default()
        },
    ));

    if let Some(chunk_id) = &state.chunk_id {
        entity.insert(ChunkMembership(chunk_id.clone()));
    }
    if let Some(rolling) = RollingResistance::from_collider(
        &definition.collider,
        physical_material.density_kg_m3,
        physical_material.rolling_resistance_coefficient,
        1.0,
    ) {
        entity.insert(rolling);
    }
    add_visual(
        &mut entity,
        &definition,
        asset_server,
        meshes,
        materials,
        mesh_cache,
        material_cache,
    );
}

fn add_visual(
    entity: &mut EntityCommands,
    definition: &ComponentDefinition,
    asset_server: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_cache: &mut HashMap<MeshKey, Handle<Mesh>>,
    material_cache: &mut HashMap<VisualMaterialKey, Handle<StandardMaterial>>,
) {
    if let Some(model_path) = &definition.visual.model {
        let (asset_path, scene_index) = match model_path.split_once('#') {
            Some((path, selector)) => {
                let index = selector
                    .strip_prefix("Scene")
                    .and_then(|value| value.parse::<usize>().ok())
                    .expect("model scene selectors are validated before app startup");
                (path, index)
            }
            None => (model_path.as_str(), 0),
        };
        let scene_path = GltfAssetLabel::Scene(scene_index).from_asset(asset_path.to_owned());
        entity.with_children(|children| {
            children.spawn((
                WorldAssetRoot(asset_server.load(scene_path)),
                Transform::default(),
            ));
        });
        return;
    }

    let mesh_key = MeshKey::from_shape(&definition.collider);
    let mesh = mesh_cache
        .entry(mesh_key)
        .or_insert_with(|| mesh_from_collider_shape(&definition.collider, meshes))
        .clone();

    let material_key = VisualMaterialKey::from_visual(&definition.visual);
    let material = material_cache
        .entry(material_key)
        .or_insert_with(|| materials.add(standard_material_from_visual(&definition.visual)))
        .clone();

    entity.insert((Mesh3d(mesh), MeshMaterial3d(material)));
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
            AlphaMode::Opaque => bevy::prelude::AlphaMode::Opaque,
            AlphaMode::Blend => bevy::prelude::AlphaMode::Blend,
        },
        ..default()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum MeshKey {
    Cuboid([u32; 3]),
    Sphere(u32),
    Capsule(u32, u32),
}

impl MeshKey {
    fn from_shape(shape: &ColliderShape) -> Self {
        match shape {
            ColliderShape::Cuboid { half_extents } => Self::Cuboid([
                half_extents[0].to_bits(),
                half_extents[1].to_bits(),
                half_extents[2].to_bits(),
            ]),
            ColliderShape::Sphere { radius } => Self::Sphere(radius.to_bits()),
            ColliderShape::Capsule {
                radius,
                segment_length,
            } => Self::Capsule(radius.to_bits(), segment_length.to_bits()),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct VisualMaterialKey {
    color: [u32; 4],
    roughness: u32,
    metallic: u32,
    alpha_mode: AlphaMode,
}

impl VisualMaterialKey {
    fn from_visual(visual: &VisualDefinition) -> Self {
        Self {
            color: visual.color_srgba.map(f32::to_bits),
            roughness: visual.perceptual_roughness.to_bits(),
            metallic: visual.metallic.to_bits(),
            alpha_mode: visual.alpha_mode,
        }
    }
}

fn collider_from_definition(shape: &ColliderShape) -> Collider {
    match shape {
        ColliderShape::Cuboid { half_extents } => Collider::cuboid(
            half_extents[0] * 2.0,
            half_extents[1] * 2.0,
            half_extents[2] * 2.0,
        ),
        ColliderShape::Sphere { radius } => Collider::sphere(*radius),
        ColliderShape::Capsule {
            radius,
            segment_length,
        } => Collider::capsule(*radius, *segment_length),
    }
}

fn mesh_from_collider_shape(shape: &ColliderShape, meshes: &mut Assets<Mesh>) -> Handle<Mesh> {
    match shape {
        ColliderShape::Cuboid { half_extents } => meshes.add(Cuboid::new(
            half_extents[0] * 2.0,
            half_extents[1] * 2.0,
            half_extents[2] * 2.0,
        )),
        ColliderShape::Sphere { radius } => meshes.add(Sphere::new(*radius).mesh().uv(48, 32)),
        ColliderShape::Capsule {
            radius,
            segment_length,
        } => meshes.add(Capsule3d::new(*radius, *segment_length)),
    }
}

fn rotation_from_degrees(rotation_xyz: [f32; 3]) -> Quat {
    Quat::from_euler(
        EulerRot::XYZ,
        rotation_xyz[0].to_radians(),
        rotation_xyz[1].to_radians(),
        rotation_xyz[2].to_radians(),
    )
}

#[cfg(test)]
mod tests {
    use super::{object_lod_enabled, static_cell_render_policy};
    use crate::config::BodyType;

    #[test]
    fn legacy_dynamic_objects_keep_rendering_when_they_move_out_of_their_source_chunk() {
        assert!(object_lod_enabled(BodyType::Static, true));
        assert!(!object_lod_enabled(BodyType::Dynamic, true));
        assert!(!object_lod_enabled(BodyType::Static, false));
    }

    #[test]
    fn destructible_and_bonded_static_platform_cells_receive_lod_without_being_merged() {
        let (lod, greedy) = static_cell_render_policy(BodyType::Static, true, true, true, true);
        assert!(lod);
        assert!(!greedy);
    }

    #[test]
    fn intact_plain_static_terrain_can_be_greedily_merged_and_rendered() {
        let (lod, greedy) = static_cell_render_policy(BodyType::Static, true, false, false, true);
        assert!(lod);
        assert!(greedy);
    }

    #[test]
    fn destroyed_or_dynamic_cells_are_not_static_lod_sources() {
        assert_eq!(
            static_cell_render_policy(BodyType::Static, false, true, true, true),
            (false, false)
        );
        assert_eq!(
            static_cell_render_policy(BodyType::Dynamic, true, false, false, true),
            (false, false)
        );
    }
}

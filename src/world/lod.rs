use std::collections::HashSet;

use bevy::{
    asset::RenderAssetUsages,
    math::DVec3,
    mesh::{Indices, Mesh, PrimitiveTopology},
    prelude::*,
};

use crate::{
    camera::FreeFlyCamera,
    config::{RuntimeConfig, WorldGeneration},
    engine::AppMode,
    physics::DestructionJournal,
    world::{WorldOrigin, generation::PLATFORM_COMPONENT, parse_voxel_instance_id},
};

/// Visual-only level-of-detail membership for every static voxel in a resident chunk.
/// Physics colliders are never replaced by these render proxies.
#[derive(Component, Clone, Debug)]
pub(crate) struct ChunkLodVisual {
    pub(crate) chunk_id: String,
    pub(crate) level: u8,
    pub(crate) max_level: u8,
    /// 0..=2 identifies a precomputed mesh-quality variant; u8::MAX is quality-independent.
    pub(crate) quality_level: u8,
    pub(crate) center: Vec3,
    pub(crate) half_extents: Vec3,
    pub(crate) near_distance_m: f32,
    pub(crate) mid_distance_m: f32,
    pub(crate) far_distance_m: f32,
    pub(crate) unload_distance_m: f32,
    /// Per-cell exact visuals shown only after a chunk fracture invalidates static snapshots.
    pub(crate) fallback_on_damage: bool,
}

pub(super) fn update_chunk_lod(
    mode: Res<State<AppMode>>,
    config: Res<RuntimeConfig>,
    camera: Query<&Transform, With<FreeFlyCamera>>,
    journal: Res<DestructionJournal>,
    mut visuals: Query<(&mut ChunkLodVisual, &mut Visibility)>,
) {
    if *mode.get() != AppMode::Simulation {
        return;
    }

    let Some(camera_position) = camera.iter().next().map(|transform| transform.translation) else {
        return;
    };

    let streaming = &config.engine.streaming;

    // Coarse proxies are snapshots of static occupancy. Once any voxel in a chunk fractures,
    // fall back to its exact LOD0 visuals so the proxy cannot resurrect broken platforms.
    let damaged_chunks: HashSet<String> = journal
        .fractured
        .iter()
        .filter_map(|id| parse_voxel_instance_id(id).map(|(chunk_id, _)| chunk_id.to_owned()))
        .collect();

    for (mut lod, mut visibility) in &mut visuals {
        if config.is_changed() {
            let chunk_extent_m = lod.half_extents.max_element() * 2.0;
            lod.near_distance_m = streaming.lod_near_distance_chunks * chunk_extent_m;
            lod.mid_distance_m = streaming.lod_mid_distance_chunks * chunk_extent_m;
            lod.far_distance_m = streaming.lod_far_distance_chunks * chunk_extent_m;
            lod.unload_distance_m =
                streaming.lod_visibility_distance_chunks as f32 * chunk_extent_m;
        }

        let distance = distance_to_aabb(
            camera_position,
            lod.center - lod.half_extents,
            lod.center + lod.half_extents,
        );
        let damaged = damaged_chunks.contains(&lod.chunk_id);
        let quality_matches = lod_quality_matches(streaming.lod_quality, lod.quality_level);
        let should_be_visible = quality_matches && lod_visual_is_visible(&lod, distance, damaged);
        visibility.set_if_neq(if should_be_visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        });
    }
}

/// Render-only stand-in for a physics body. It deliberately carries no Avian components.
#[derive(Component, Clone, Copy)]
pub(super) struct BodyLodProxy {
    pub(super) owner: Entity,
}

/// The physics body owns its unchanged collider/simulation state; this reference selects a
/// separate render-only proxy when the owner moves outside the full-detail radius.
#[derive(Component, Clone, Copy)]
pub(super) struct BodyLodOwner {
    pub(super) proxy: Entity,
    pub(super) radius_m: f32,
}

pub(super) fn update_physics_body_lod(
    mode: Res<State<AppMode>>,
    config: Res<RuntimeConfig>,
    camera: Query<&Transform, With<FreeFlyCamera>>,
    mut owners: Query<
        (Entity, &BodyLodOwner, &Transform, &mut Visibility),
        (With<BodyLodOwner>, Without<BodyLodProxy>),
    >,
    mut proxies: Query<
        (&BodyLodProxy, &mut Transform, &mut Visibility),
        (With<BodyLodProxy>, Without<BodyLodOwner>),
    >,
) {
    if *mode.get() != AppMode::Simulation {
        return;
    }
    if !config.engine.streaming.lod_enabled {
        for (_, _, _, mut visibility) in &mut owners {
            visibility.set_if_neq(Visibility::Visible);
        }
        for (_, _, mut visibility) in &mut proxies {
            visibility.set_if_neq(Visibility::Hidden);
        }
        return;
    }
    let Some(camera_position) = camera.iter().next().map(|transform| transform.translation) else {
        return;
    };

    for (owner, body, body_transform, mut body_visibility) in &mut owners {
        let Ok((proxy, mut proxy_transform, mut proxy_visibility)) = proxies.get_mut(body.proxy)
        else {
            continue;
        };
        if proxy.owner != owner {
            continue;
        }
        proxy_transform.translation = body_transform.translation;
        proxy_transform.rotation = body_transform.rotation;
        proxy_transform.scale = body_transform.scale;

        let distance = camera_position.distance(body_transform.translation);
        let full_detail_radius = config.engine.streaming.load_radius_chunks as f32 * body.radius_m;
        let lod_visibility_radius =
            config.engine.streaming.lod_visibility_distance_chunks as f32 * body.radius_m;

        // Hysteresis keeps one representation active through the threshold band. The owner
        // and proxy are switched in one update, so the pair is never rendered simultaneously.
        let (show_full_detail, show_lod) = body_lod_visibility(
            distance,
            full_detail_radius,
            lod_visibility_radius,
            *body_visibility != Visibility::Hidden,
            *proxy_visibility == Visibility::Visible,
        );

        body_visibility.set_if_neq(if show_full_detail {
            Visibility::Visible
        } else {
            Visibility::Hidden
        });
        proxy_visibility.set_if_neq(if show_lod {
            Visibility::Visible
        } else {
            Visibility::Hidden
        });
    }
}

#[derive(Component)]
struct FarPlatformLod {
    world_size_units: u64,
    render_distance_chunks: u32,
    lod_distance_chunks: u32,
    anchor_absolute: DVec3,
}

/// Keeps the far procedural platform visible as one camera-relative surface mesh. It is
/// independent of chunk residency and contains no physics components or per-chunk entities.
pub(super) fn update_far_platform_lod(
    mode: Res<State<AppMode>>,
    config: Res<RuntimeConfig>,
    world_origin: Res<WorldOrigin>,
    camera: Query<&Transform, With<FreeFlyCamera>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut far_platforms: Query<(
        Entity,
        &mut FarPlatformLod,
        &mut Mesh3d,
        &mut MeshMaterial3d<StandardMaterial>,
        &mut Transform,
        &mut Visibility,
    )>,
) {
    let is_simulation = *mode.get() == AppMode::Simulation && config.engine.streaming.lod_enabled;
    let generation = config.world.generation.as_ref();
    let Some(WorldGeneration::FlatPlatform { size_units, .. }) =
        generation.filter(|_| is_simulation)
    else {
        for (_, _, _, _, _, mut visibility) in &mut far_platforms {
            visibility.set_if_neq(Visibility::Hidden);
        }
        return;
    };
    let Some(camera_transform) = camera.iter().next() else {
        return;
    };

    let camera_absolute = world_origin.0 + camera_transform.translation.as_dvec3();
    let render_distance = config.engine.streaming.load_radius_chunks;
    let lod_distance = config.engine.streaming.lod_visibility_distance_chunks;
    let visual = config
        .components
        .definitions
        .get(PLATFORM_COMPONENT)
        .map(|definition| &definition.visual);
    let Some(visual) = visual else {
        warn!("Far platform LOD is unavailable because the platform visual definition is missing");
        return;
    };

    // A 32m x 32m chunk has a half-diagonal of 22.6m. Start beyond the outer
    // boundary of the full-detail chunk set, with enough overlap for the streaming tick.
    let inner_radius_m = render_distance as f64 * 32.0 + 32.0 * std::f64::consts::FRAC_1_SQRT_2;
    let outer_radius_m = lod_distance as f64 * 32.0;
    if outer_radius_m <= inner_radius_m {
        for (_, _, _, _, _, mut visibility) in &mut far_platforms {
            visibility.set_if_neq(Visibility::Hidden);
        }
        return;
    }

    let new_material = || StandardMaterial {
        base_color: Color::srgba(
            visual.color_srgba[0],
            visual.color_srgba[1],
            visual.color_srgba[2],
            visual.color_srgba[3],
        ),
        perceptual_roughness: visual.perceptual_roughness,
        metallic: visual.metallic,
        alpha_mode: match visual.alpha_mode {
            crate::config::AlphaMode::Opaque => AlphaMode::Opaque,
            crate::config::AlphaMode::Blend => AlphaMode::Blend,
        },
        ..default()
    };

    if let Some((_, mut far, mut mesh_handle, mut material_handle, mut transform, mut visibility)) =
        far_platforms.iter_mut().next()
    {
        let moved = camera_absolute.distance(far.anchor_absolute) >= 16.0;
        let changed = config.is_changed()
            || moved
            || far.world_size_units != *size_units
            || far.render_distance_chunks != render_distance
            || far.lod_distance_chunks != lod_distance;
        if changed {
            let mesh = flat_platform_lod_ring(
                *size_units,
                camera_absolute,
                inner_radius_m,
                outer_radius_m,
                config.engine.streaming.lod_geometry_errors_m
                    [config.engine.streaming.lod_quality as usize] as f64,
            );
            mesh_handle.0 = meshes.add(mesh);
            material_handle.0 = materials.add(new_material());
            far.world_size_units = *size_units;
            far.render_distance_chunks = render_distance;
            far.lod_distance_chunks = lod_distance;
            far.anchor_absolute = camera_absolute;
        }
        if world_origin.is_changed() || changed {
            transform.translation = (far.anchor_absolute - world_origin.0).as_vec3();
        }
        visibility.set_if_neq(Visibility::Visible);
    } else {
        let mesh = flat_platform_lod_ring(
            *size_units,
            camera_absolute,
            inner_radius_m,
            outer_radius_m,
            config.engine.streaming.lod_geometry_errors_m
                [config.engine.streaming.lod_quality as usize] as f64,
        );
        commands.spawn((
            Name::new("procedural:flat-platform:far-lod"),
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(materials.add(new_material())),
            Transform::from_translation((camera_absolute - world_origin.0).as_vec3()),
            Visibility::Visible,
            FarPlatformLod {
                world_size_units: *size_units,
                render_distance_chunks: render_distance,
                lod_distance_chunks: lod_distance,
                anchor_absolute: camera_absolute,
            },
        ));
    }
}

fn flat_platform_lod_ring(
    size_units: u64,
    anchor_absolute: DVec3,
    inner_radius_m: f64,
    outer_radius_m: f64,
    max_error_m: f64,
) -> Mesh {
    let half_extent = size_units as f64 * 0.5;
    let mut angles = (0..512)
        .map(|index| index as f64 * std::f64::consts::TAU / 512.0)
        .collect::<Vec<_>>();
    for corner_x in [-half_extent, half_extent] {
        for corner_z in [-half_extent, half_extent] {
            angles.push(
                (corner_z - anchor_absolute.z)
                    .atan2(corner_x - anchor_absolute.x)
                    .rem_euclid(std::f64::consts::TAU),
            );
        }
    }
    angles.sort_by(f64::total_cmp);
    angles.dedup_by(|left, right| (*left - *right).abs() < 1.0e-10);

    // Sagitta-driven tessellation keeps the radial coverage boundary within the selected
    // geometric error budget. The hard cap bounds vertex memory at maximum LOD distance.
    let segments = (std::f64::consts::PI * (outer_radius_m / (2.0 * max_error_m.max(0.01))).sqrt())
        .ceil()
        .clamp(256.0, 65_536.0) as usize;
    let mut uniform_angles = (0..segments)
        .map(|index| index as f64 * std::f64::consts::TAU / segments as f64)
        .collect::<Vec<_>>();
    angles.append(&mut uniform_angles);
    angles.sort_by(f64::total_cmp);
    angles.dedup_by(|left, right| (*left - *right).abs() < 1.0e-10);

    let mut positions = Vec::<[f32; 3]>::with_capacity(angles.len() * 12);
    let mut normals = Vec::<[f32; 3]>::with_capacity(angles.len() * 12);
    let mut uvs = Vec::<[f32; 2]>::with_capacity(angles.len() * 12);
    let mut indices = Vec::<u32>::with_capacity(angles.len() * 6);
    let base_y = (1.0 - anchor_absolute.y) as f32 - 0.0002;

    for index in 0..angles.len() {
        let angle0 = angles[index];
        let angle1 = if index + 1 == angles.len() {
            angles[0] + std::f64::consts::TAU
        } else {
            angles[index + 1]
        };
        let direction0 = (angle0.cos(), angle0.sin());
        let direction1 = (angle1.cos(), angle1.sin());
        let edge_distance = |direction: (f64, f64)| {
            let x_distance = if direction.0 > 1.0e-12 {
                (half_extent - anchor_absolute.x) / direction.0
            } else if direction.0 < -1.0e-12 {
                (-half_extent - anchor_absolute.x) / direction.0
            } else {
                f64::INFINITY
            };
            let z_distance = if direction.1 > 1.0e-12 {
                (half_extent - anchor_absolute.z) / direction.1
            } else if direction.1 < -1.0e-12 {
                (-half_extent - anchor_absolute.z) / direction.1
            } else {
                f64::INFINITY
            };
            x_distance.min(z_distance)
        };
        let outer0 = outer_radius_m.min(edge_distance(direction0));
        let outer1 = outer_radius_m.min(edge_distance(direction1));
        if outer0 <= inner_radius_m || outer1 <= inner_radius_m {
            continue;
        }

        let point = |direction: (f64, f64), radius: f64| {
            [
                (direction.0 * radius) as f32,
                base_y,
                (direction.1 * radius) as f32,
            ]
        };
        let quad = [
            point(direction0, inner_radius_m),
            point(direction1, inner_radius_m),
            point(direction1, outer1),
            point(direction0, outer0),
        ];
        let base = positions.len() as u32;
        positions.extend(quad);
        normals.extend([[0.0, 1.0, 0.0]; 4]);
        let angular_uv = [angle0 as f32, angle1 as f32];
        uvs.extend([
            [angular_uv[0], inner_radius_m as f32],
            [angular_uv[1], inner_radius_m as f32],
            [angular_uv[1], outer1 as f32],
            [angular_uv[0], outer0 as f32],
        ]);
        // Counter-clockwise winding when viewed from above (+Y).
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

fn body_lod_visibility(
    distance: f32,
    full_detail_radius: f32,
    lod_visibility_radius: f32,
    was_full_detail: bool,
    was_lod_visible: bool,
) -> (bool, bool) {
    let full_detail = distance <= full_detail_radius
        || (was_full_detail && distance <= full_detail_radius * 1.05);
    let lod_limit = lod_visibility_radius * if was_lod_visible { 1.05 } else { 1.0 };
    (full_detail, !full_detail && distance <= lod_limit)
}

fn lod_quality_matches(active_quality: u8, mesh_quality: u8) -> bool {
    mesh_quality == u8::MAX || mesh_quality == active_quality
}

fn distance_to_aabb(point: Vec3, min: Vec3, max: Vec3) -> f32 {
    point.distance(point.clamp(min, max))
}

fn lod_visual_is_visible(lod: &ChunkLodVisual, distance: f32, damaged: bool) -> bool {
    if lod.fallback_on_damage {
        damaged && distance <= lod.unload_distance_m
    } else if damaged {
        false
    } else {
        lod_level_is_visible(
            lod.level,
            lod.max_level,
            distance,
            lod.near_distance_m,
            lod.mid_distance_m,
            lod.far_distance_m,
            lod.unload_distance_m,
        )
    }
}

fn lod_level_is_visible(
    level: u8,
    max_level: u8,
    distance: f32,
    near: f32,
    mid: f32,
    far: f32,
    unload: f32,
) -> bool {
    match level {
        0 if max_level == 0 => distance <= unload,
        0 => distance <= near,
        1 if max_level == 1 => distance > near && distance <= unload,
        1 => distance > near && distance <= mid,
        2 if max_level == 2 => distance > mid && distance <= unload,
        2 => distance > mid && distance <= far,
        3 if max_level >= 3 => distance > far && distance <= unload,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ChunkLodVisual, body_lod_visibility, distance_to_aabb, flat_platform_lod_ring,
        lod_level_is_visible, lod_quality_matches, lod_visual_is_visible,
    };
    use bevy::prelude::Vec3;
    use bevy::{
        math::DVec3,
        mesh::{Mesh, VertexAttributeValues},
    };

    #[test]
    fn lod_distance_uses_chunk_bounds_not_only_the_chunk_center() {
        let min = Vec3::new(-16.0, 0.0, -16.0);
        let max = Vec3::new(0.0, 16.0, 0.0);
        assert_eq!(distance_to_aabb(Vec3::new(-1.0, 8.0, -8.0), min, max), 0.0);
        assert!((distance_to_aabb(Vec3::new(3.0, 8.0, -8.0), min, max) - 3.0).abs() < 1.0e-5);
        assert!(
            (distance_to_aabb(Vec3::new(3.0, 20.0, 4.0), min, max) - 41.0_f32.sqrt()).abs()
                < 1.0e-5
        );
    }

    #[test]
    fn exact_per_cell_visuals_replace_static_snapshots_only_after_damage() {
        let fallback = ChunkLodVisual {
            chunk_id: "chunk_platform".to_owned(),
            level: 0,
            max_level: 0,
            quality_level: u8::MAX,
            center: Vec3::ZERO,
            half_extents: Vec3::splat(8.0),
            near_distance_m: 16.0,
            mid_distance_m: 28.0,
            far_distance_m: 40.0,
            unload_distance_m: 48.0,
            fallback_on_damage: true,
        };
        let snapshot = ChunkLodVisual {
            fallback_on_damage: false,
            ..fallback.clone()
        };

        assert!(!lod_visual_is_visible(&fallback, 4.0, false));
        assert!(lod_visual_is_visible(&fallback, 4.0, true));
        assert!(!lod_visual_is_visible(&fallback, 48.1, true));
        assert!(lod_visual_is_visible(&snapshot, 4.0, false));
        assert!(!lod_visual_is_visible(&snapshot, 4.0, true));
    }

    #[test]
    fn physics_body_lod_switches_are_hysteretic_and_never_overlap() {
        let (full, lod) = body_lod_visibility(10.0, 10.0, 100.0, false, false);
        assert!(full);
        assert!(!lod);

        let (full, lod) = body_lod_visibility(10.4, 10.0, 100.0, true, false);
        assert!(full);
        assert!(!lod);

        let (full, lod) = body_lod_visibility(10.6, 10.0, 100.0, true, false);
        assert!(!full);
        assert!(lod);

        let (full, lod) = body_lod_visibility(105.0, 10.0, 100.0, false, true);
        assert!(!full);
        assert!(lod);

        let (full, lod) = body_lod_visibility(106.0, 10.0, 100.0, false, true);
        assert!(!full);
        assert!(!lod);
        assert!(!(full && lod));
    }

    #[test]
    fn active_quality_selects_only_its_precomputed_mesh_variant() {
        assert!(lod_quality_matches(0, 0));
        assert!(!lod_quality_matches(0, 1));
        assert!(lod_quality_matches(1, 1));
        assert!(!lod_quality_matches(1, 2));
        assert!(lod_quality_matches(2, 2));
        assert!(lod_quality_matches(2, u8::MAX));
    }

    #[test]
    fn four_lod_levels_use_non_overlapping_distance_bands() {
        let near = 10.0;
        let mid = 17.5;
        let far = 25.0;
        let unload = 30.0;

        assert!(lod_level_is_visible(0, 3, near, near, mid, far, unload));
        assert!(!lod_level_is_visible(
            0,
            3,
            near + 0.1,
            near,
            mid,
            far,
            unload
        ));
        assert!(lod_level_is_visible(
            1,
            3,
            near + 0.1,
            near,
            mid,
            far,
            unload
        ));
        assert!(lod_level_is_visible(1, 3, mid, near, mid, far, unload));
        assert!(!lod_level_is_visible(
            1,
            3,
            mid + 0.1,
            near,
            mid,
            far,
            unload
        ));
        assert!(lod_level_is_visible(
            2,
            3,
            mid + 0.1,
            near,
            mid,
            far,
            unload
        ));
        assert!(lod_level_is_visible(2, 3, far, near, mid, far, unload));
        assert!(!lod_level_is_visible(
            2,
            3,
            far + 0.1,
            near,
            mid,
            far,
            unload
        ));
        assert!(lod_level_is_visible(
            3,
            3,
            far + 0.1,
            near,
            mid,
            far,
            unload
        ));
        assert!(lod_level_is_visible(3, 3, unload, near, mid, far, unload));
        assert!(!lod_level_is_visible(
            3,
            3,
            unload + 0.1,
            near,
            mid,
            far,
            unload
        ));
    }

    #[test]
    fn flat_platform_far_lod_covers_500000_chunks_with_bounded_mesh_size() {
        let mesh = flat_platform_lod_ring(255_000_000, DVec3::ZERO, 4_000.0, 16_000_000.0, 0.25);
        let VertexAttributeValues::Float32x3(positions) =
            mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap()
        else {
            panic!("far-field surface must expose float3 positions");
        };

        assert!(!positions.is_empty());
        assert!(positions.len() <= (65_536 + 520) * 4);
        assert!(
            positions
                .iter()
                .all(|position| (position[1] - 0.9998).abs() < 1.0e-4)
        );
        let max_radius = positions
            .iter()
            .map(|position| position[0].hypot(position[2]))
            .fold(0.0_f32, f32::max);
        assert!((max_radius - 16_000_000.0).abs() < 2.0);
        assert!(positions.iter().all(
            |position| position[0].abs() <= 127_500_000.0 && position[2].abs() <= 127_500_000.0
        ));
    }

    #[test]
    fn lower_resolution_chunks_keep_the_best_available_level_until_unload() {
        let (near, mid, far, unload) = (10.0, 17.5, 25.0, 30.0);
        assert!(lod_level_is_visible(0, 0, 20.0, near, mid, far, unload));
        assert!(!lod_level_is_visible(0, 0, 30.1, near, mid, far, unload));
        assert!(lod_level_is_visible(1, 1, 29.9, near, mid, far, unload));
        assert!(!lod_level_is_visible(1, 1, 30.1, near, mid, far, unload));
        assert!(lod_level_is_visible(2, 2, 29.9, near, mid, far, unload));
    }
}

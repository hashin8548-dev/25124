use std::collections::HashSet;

use bevy::prelude::*;

use crate::{
    camera::FreeFlyCamera, config::RuntimeConfig, engine::AppMode, physics::DestructionJournal,
    world::parse_voxel_instance_id,
};

/// Visual-only level-of-detail membership for every static voxel in a resident chunk.
/// Physics colliders are never replaced by these render proxies.
#[derive(Component, Clone, Debug)]
pub(crate) struct ChunkLodVisual {
    pub(crate) chunk_id: String,
    pub(crate) level: u8,
    pub(crate) max_level: u8,
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
            lod.unload_distance_m = streaming.unload_radius_chunks as f32 * chunk_extent_m;
        }

        let distance = distance_to_aabb(
            camera_position,
            lod.center - lod.half_extents,
            lod.center + lod.half_extents,
        );
        let damaged = damaged_chunks.contains(&lod.chunk_id);
        let should_be_visible = lod_visual_is_visible(lod, distance, damaged);
        visibility.set_if_neq(if should_be_visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        });
    }
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
    use super::{ChunkLodVisual, distance_to_aabb, lod_level_is_visible, lod_visual_is_visible};
    use bevy::prelude::Vec3;

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
    fn lower_resolution_chunks_keep_the_best_available_level_until_unload() {
        let (near, mid, far, unload) = (10.0, 17.5, 25.0, 30.0);
        assert!(lod_level_is_visible(0, 0, 20.0, near, mid, far, unload));
        assert!(!lod_level_is_visible(0, 0, 30.1, near, mid, far, unload));
        assert!(lod_level_is_visible(1, 1, 29.9, near, mid, far, unload));
        assert!(!lod_level_is_visible(1, 1, 30.1, near, mid, far, unload));
        assert!(lod_level_is_visible(2, 2, 29.9, near, mid, far, unload));
    }
}

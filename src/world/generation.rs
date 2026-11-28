use std::path::PathBuf;

use anyhow::{Result, ensure};
use bevy::{ecs::system::SystemParam, math::DVec3, prelude::*};

use crate::config::{ChunkFile, ChunkLayer, ChunkSource, WorldGeneration};

const CHUNK_EDGE: i64 = 32;
const CHUNK_HEIGHT: u32 = 8;
const PLATFORM_COMPONENT: &str = "block.bedrock";

/// Absolute world-space origin used to keep Bevy's f32 transforms close to the camera.
#[derive(Resource, Clone, Copy, Debug, Default)]
pub(crate) struct WorldOrigin(pub DVec3);

type RebaseCameraQuery<'w, 's> =
    Query<'w, 's, &'static Transform, With<crate::camera::FreeFlyCamera>>;
type RebaseEntityQuery<'w, 's> = Query<
    'w,
    's,
    (
        &'static mut Transform,
        Option<&'static mut crate::world::lod::ChunkLodVisual>,
    ),
    Without<ChildOf>,
>;

#[derive(SystemParam)]
struct WorldRebaseParams<'w, 's> {
    queries: ParamSet<'w, 's, (RebaseCameraQuery<'w, 's>, RebaseEntityQuery<'w, 's>)>,
}

/// Recenters the f32 simulation around the camera while preserving absolute world position.
/// Rebase steps are exact multiples of the procedural chunk edge, so voxel boundaries remain stable.
pub(super) fn rebase_world_origin(
    mut origin: ResMut<WorldOrigin>,
    mut world_state: ResMut<crate::persistence::WorldStateFile>,
    mut params: WorldRebaseParams,
) {
    let Some(camera_position) = params.queries
        .p0()
        .iter()
        .next()
        .map(|transform| transform.translation)
    else {
        return;
    };
    if camera_position.abs().max_element() <= 4096.0 {
        return;
    }

    let shift = (camera_position / CHUNK_EDGE as f32 / 32.0).round() * (CHUNK_EDGE as f32 * 32.0);
    if shift == bevy::prelude::Vec3::ZERO {
        return;
    }

    {
        let mut all_transforms = params.queries.p1();
        for (mut transform, lod_visual) in &mut all_transforms {
            transform.translation -= shift;
            if let Some(mut visual) = lod_visual {
                visual.center -= shift;
            }
        }
    }

    for state in world_state.bodies.values_mut() {
        state.translation_m =
            (bevy::prelude::Vec3::from_array(state.translation_m) - shift).to_array();
    }
    for state in world_state.fragments.values_mut() {
        state.translation_m =
            (bevy::prelude::Vec3::from_array(state.translation_m) - shift).to_array();
    }

    origin.0 += shift.as_dvec3();
    world_state.origin_m = origin.0.to_array();
    info!(
        "Rebased world origin by {:?}; absolute origin is {:?}",
        shift, origin.0
    );
}

/// Produces the bounded set of platform chunks near the absolute camera position.
/// Coordinates are integer chunk addresses; no world-sized allocation or manifest is created.
pub(crate) fn platform_sources_near(
    camera_absolute: DVec3,
    generation: &WorldGeneration,
    load_radius_chunks: u32,
) -> Vec<ChunkSource> {
    let WorldGeneration::FlatPlatform { size_units, .. } = generation;
    let size = *size_units as i64;
    let min_cell = -(size / 2);
    let max_cell = min_cell + size;
    let min_chunk = min_cell.div_euclid(CHUNK_EDGE);
    let max_chunk = (max_cell - 1).div_euclid(CHUNK_EDGE);

    let camera_x = (camera_absolute.x / CHUNK_EDGE as f64).floor() as i64;
    let camera_z = (camera_absolute.z / CHUNK_EDGE as f64).floor() as i64;
    let search_radius = i64::from(load_radius_chunks).saturating_add(1);

    let mut sources = Vec::new();
    for chunk_z in (camera_z - search_radius)..=(camera_z + search_radius) {
        if !(min_chunk..=max_chunk).contains(&chunk_z) {
            continue;
        }
        for chunk_x in (camera_x - search_radius)..=(camera_x + search_radius) {
            if !(min_chunk..=max_chunk).contains(&chunk_x) {
                continue;
            }
            if let (Ok(x), Ok(z)) = (i32::try_from(chunk_x), i32::try_from(chunk_z)) {
                let id = format!("platform_{x}_{z}");
                sources.push(ChunkSource {
                    id,
                    region_id: "procedural_flat_platform".to_owned(),
                    region_coordinate: [0, 0],
                    coordinate: [x, 0, z],
                    dimensions: [32, CHUNK_HEIGHT, 32],
                    voxel_size_m: 1.0,
                    path: PathBuf::new(),
                    generated: true,
                });
            }
        }
    }
    sources
}

/// Deterministically synthesizes a flat bedrock slab and its raised perimeter for one chunk.
pub(crate) fn generate_platform_chunk(
    source: &ChunkSource,
    generation: &WorldGeneration,
) -> Result<ChunkFile> {
    let WorldGeneration::FlatPlatform {
        size_units,
        border_height_units,
        border_thickness_units,
    } = generation;

    ensure!(source.generated, "source '{}' is not procedural", source.id);
    ensure!(
        source.dimensions == [32, CHUNK_HEIGHT, 32],
        "unexpected procedural chunk dimensions"
    );
    ensure!(
        *size_units <= 255_000_000 && *size_units >= 32 && size_units % 2 == 0,
        "flat platform size must be an even value in [32, 255000000]"
    );
    ensure!(
        (1..CHUNK_HEIGHT).contains(border_height_units),
        "border height must be in [1, {}]",
        CHUNK_HEIGHT - 1
    );
    ensure!(
        (1..=16).contains(border_thickness_units),
        "border thickness must be in [1, 16]"
    );

    let size = *size_units as i64;
    let min_cell = -(size / 2);
    let max_cell = min_cell + size;
    let border_min = min_cell + i64::from(*border_thickness_units);
    let border_max = max_cell - i64::from(*border_thickness_units);

    let origin_x = i64::from(source.coordinate[0]) * CHUNK_EDGE;
    let origin_z = i64::from(source.coordinate[2]) * CHUNK_EDGE;
    let mut layers = Vec::new();

    for y in 0..=*border_height_units {
        let mut rows = Vec::with_capacity(32);
        let mut layer_has_cells = false;
        for z in 0..32_i64 {
            let global_z = origin_z + z;
            let mut row = String::with_capacity(32);
            for x in 0..32_i64 {
                let global_x = origin_x + x;
                let inside_platform = (min_cell..max_cell).contains(&global_x)
                    && (min_cell..max_cell).contains(&global_z);
                let border_cell = inside_platform
                    && (global_x < border_min
                        || global_x >= border_max
                        || global_z < border_min
                        || global_z >= border_max);
                let occupied = inside_platform && (y == 0 || (y > 0 && border_cell));
                row.push(if occupied { '1' } else { '0' });
                layer_has_cells |= occupied;
            }
            rows.push(row);
        }

        if layer_has_cells {
            layers.push(ChunkLayer { y, rows });
        }
    }

    Ok(ChunkFile {
        schema_version: 1,
        id: source.id.clone(),
        coordinate: source.coordinate,
        dimensions: source.dimensions,
        voxel_size_m: source.voxel_size_m,
        palette: vec!["air".to_owned(), PLATFORM_COMPONENT.to_owned()],
        layers,
    })
}

#[cfg(test)]
mod tests {
    use super::{generate_platform_chunk, platform_sources_near};
    use crate::config::WorldGeneration;
    use bevy::math::DVec3;

    fn generation(size_units: u64) -> WorldGeneration {
        WorldGeneration::FlatPlatform {
            size_units,
            border_height_units: 4,
            border_thickness_units: 2,
        }
    }

    #[test]
    fn generator_creates_flat_interior_and_continuous_perimeter() {
        let recipe = generation(256);
        let sources = platform_sources_near(DVec3::new(16.0, 0.0, -112.0), &recipe, 0);
        let center = sources
            .iter()
            .find(|source| source.coordinate == [0, 0, -4])
            .unwrap();
        let chunk = generate_platform_chunk(center, &recipe).unwrap();

        assert_eq!(chunk.dimensions, [32, 8, 32]);
        assert_eq!(chunk.layers[0].y, 0);
        let expected_filled_row = "1".repeat(32);
        assert!(
            chunk.layers[0]
                .rows
                .iter()
                .all(|row| row == &expected_filled_row)
        );
        assert!(chunk.layers.iter().any(|layer| layer.y == 4));
        let top = chunk.layers.iter().find(|layer| layer.y == 4).unwrap();
        assert!(top.rows[0].chars().all(|cell| cell == '1'));
        assert!(top.rows[16].chars().all(|cell| cell == '0'));
    }

    #[test]
    fn generator_is_bounded_and_uses_integer_coordinates_at_maximum_size() {
        let recipe = generation(255_000_000);
        let edge_x = ((127_499_999_i64).div_euclid(32)) as f64;
        let sources = platform_sources_near(DVec3::new(edge_x * 32.0, 0.0, 0.0), &recipe, 2);
        assert!(sources.len() <= 36);
        assert!(sources.iter().all(|source| source.generated));
        assert!(
            sources
                .iter()
                .any(|source| source.coordinate[0] > 3_900_000)
        );
    }

    #[test]
    fn chunks_outside_the_platform_are_not_generated() {
        let recipe = generation(256);
        let sources = platform_sources_near(DVec3::new(1000.0, 0.0, 1000.0), &recipe, 2);
        assert!(sources.is_empty());
    }
}

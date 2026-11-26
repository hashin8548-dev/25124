use std::collections::HashSet;

use anyhow::{Result, ensure};

use super::load::ChunkSource;
use super::schema::{
    AlphaMode, BodyType, ChunkFile, ColliderShape, ComponentCatalog, EngineSettings,
    MaterialCatalog, WorldManifest,
};

const SCHEMA_VERSION: u32 = 1;
const ENGINE_SCHEMA_VERSION: u32 = 2;
const CAMERA_PITCH_LIMIT_DEGREES: f32 = 89.0;

pub fn validate_all(
    engine: &EngineSettings,
    materials: &MaterialCatalog,
    components: &ComponentCatalog,
    world: &WorldManifest,
    chunks: &[ChunkSource],
) -> Result<()> {
    ensure!(
        engine.schema_version == ENGINE_SCHEMA_VERSION,
        "unsupported engine schema version {}",
        engine.schema_version
    );
    ensure!(
        materials.schema_version == SCHEMA_VERSION,
        "unsupported material catalogue schema version {}",
        materials.schema_version
    );
    ensure!(
        components.schema_version == SCHEMA_VERSION,
        "unsupported component catalogue schema version {}",
        components.schema_version
    );
    ensure!(
        world.schema_version == SCHEMA_VERSION,
        "unsupported world schema version {}",
        world.schema_version
    );

    let physics = &engine.physics;
    ensure!(
        physics.meters_per_unit.is_finite()
            && (physics.meters_per_unit - 1.0).abs() <= f32::EPSILON,
        "physics.meters_per_unit must be exactly 1.0 (one world unit is one metre)"
    );
    ensure!(
        physics.gravity_m_s2.iter().all(|value| value.is_finite()),
        "gravity_m_s2 must contain only finite values"
    );
    ensure!(
        physics.fixed_hz.is_finite() && (30.0..=480.0).contains(&physics.fixed_hz),
        "physics.fixed_hz must be between 30 and 480 Hz"
    );
    ensure!(
        (1..=64).contains(&physics.substeps),
        "physics.substeps must be between 1 and 64"
    );
    ensure!(
        physics.restitution_threshold_m_s.is_finite() && physics.restitution_threshold_m_s >= 0.0,
        "physics.restitution_threshold_m_s must be finite and non-negative"
    );
    ensure!(
        (1..=16).contains(&physics.restitution_iterations),
        "physics.restitution_iterations must be between 1 and 16"
    );
    ensure!(
        physics.contact_damping_ratio.is_finite()
            && (0.1..=20.0).contains(&physics.contact_damping_ratio),
        "physics.contact_damping_ratio must be between 0.1 and 20"
    );
    ensure!(
        physics.contact_frequency_factor.is_finite()
            && (0.1..=10.0).contains(&physics.contact_frequency_factor),
        "physics.contact_frequency_factor must be between 0.1 and 10"
    );
    ensure!(
        physics.max_overlap_solve_speed_m_s.is_finite()
            && physics.max_overlap_solve_speed_m_s > 0.0,
        "physics.max_overlap_solve_speed_m_s must be positive and finite"
    );
    ensure!(
        physics.warm_start_coefficient.is_finite()
            && (0.0..=1.0).contains(&physics.warm_start_coefficient),
        "physics.warm_start_coefficient must be in [0, 1]"
    );
    ensure!(
        physics.sleep_linear_threshold_m_s.is_finite() && physics.sleep_linear_threshold_m_s > 0.0,
        "physics.sleep_linear_threshold_m_s must be positive and finite"
    );
    ensure!(
        physics.sleep_angular_threshold_rad_s.is_finite()
            && physics.sleep_angular_threshold_rad_s > 0.0,
        "physics.sleep_angular_threshold_rad_s must be positive and finite"
    );
    ensure!(
        physics.sleep_after_seconds.is_finite() && physics.sleep_after_seconds > 0.0,
        "physics.sleep_after_seconds must be positive and finite"
    );

    let camera = &engine.camera;
    ensure!(
        camera
            .spawn_position_m
            .iter()
            .all(|value| value.is_finite()),
        "camera.spawn_position_m must contain only finite values"
    );
    ensure!(camera.yaw_degrees.is_finite(), "camera yaw must be finite");
    ensure!(
        camera.pitch_degrees.is_finite()
            && camera.pitch_degrees.abs() <= CAMERA_PITCH_LIMIT_DEGREES,
        "camera pitch must be between -89 and 89 degrees"
    );
    ensure!(
        camera.move_speed_m_s.is_finite() && camera.move_speed_m_s > 0.0,
        "camera move speed must be positive and finite"
    );
    ensure!(
        camera.sprint_multiplier.is_finite() && camera.sprint_multiplier >= 1.0,
        "camera sprint multiplier must be finite and at least 1.0"
    );
    ensure!(
        camera.mouse_radians_per_pixel.is_finite() && camera.mouse_radians_per_pixel >= 0.0,
        "camera mouse sensitivity must be finite and non-negative"
    );

    ensure!(
        engine.rendering.key_light_illuminance_lux.is_finite()
            && engine.rendering.key_light_illuminance_lux > 0.0,
        "key light illuminance must be positive and finite"
    );
    ensure!(
        engine
            .rendering
            .key_light_rotation_degrees_xyz
            .iter()
            .all(|value| value.is_finite()),
        "key light rotation must contain only finite values"
    );

    let streaming = &engine.streaming;
    ensure!(
        streaming.unload_radius_chunks > streaming.load_radius_chunks,
        "streaming.unload_radius_chunks must exceed load_radius_chunks to provide hysteresis"
    );
    ensure!(
        (1..=16).contains(&streaming.max_parallel_chunk_loads),
        "streaming.max_parallel_chunk_loads must be between 1 and 16"
    );
    ensure!(
        (1..=256).contains(&streaming.max_chunk_spawns_per_frame),
        "streaming.max_chunk_spawns_per_frame must be between 1 and 256"
    );

    ensure!(
        !materials.materials.is_empty(),
        "material catalogue is empty"
    );
    for (id, material) in &materials.materials {
        ensure!(!id.trim().is_empty(), "material ids cannot be empty");
        ensure!(
            material.density_kg_m3.is_finite() && material.density_kg_m3 > 0.0,
            "material '{id}' density must be positive and finite"
        );
        ensure!(
            material.static_friction.is_finite() && material.static_friction >= 0.0,
            "material '{id}' static friction must be finite and non-negative"
        );
        ensure!(
            material.dynamic_friction.is_finite() && material.dynamic_friction >= 0.0,
            "material '{id}' dynamic friction must be finite and non-negative"
        );
        ensure!(
            material.restitution.is_finite() && (0.0..=1.0).contains(&material.restitution),
            "material '{id}' restitution must be in [0, 1]"
        );
        ensure!(
            material.rolling_resistance_coefficient.is_finite()
                && (0.0..=1.0).contains(&material.rolling_resistance_coefficient),
            "material '{id}' rolling resistance coefficient must be in [0, 1]"
        );
    }

    ensure!(
        !components.definitions.is_empty(),
        "component catalogue is empty"
    );
    for (id, definition) in &components.definitions {
        ensure!(
            !id.trim().is_empty(),
            "component definition ids cannot be empty"
        );
        ensure!(
            materials.materials.contains_key(&definition.material),
            "component definition '{id}' references unknown material '{}'",
            definition.material
        );
        validate_collider(&definition.collider, id)?;
        validate_visual(&definition.visual, id)?;
        if let Some(ccd) = &definition.swept_ccd {
            ensure!(
                matches!(definition.body_type, BodyType::Dynamic),
                "static component definition '{id}' cannot configure swept CCD"
            );
            ensure!(
                ccd.threshold.is_finite() && (0.0..=1.0).contains(&ccd.threshold),
                "component definition '{id}' swept CCD threshold must be finite and in [0, 1]"
            );
        }
        if let Some(bond_strength) = &definition.bond_strength {
            ensure!(
                matches!(&definition.collider, ColliderShape::Cuboid { .. }),
                "bonded component definition '{id}' must use a cuboid collider"
            );
            ensure!(
                [
                    bond_strength.tensile_strength_pa,
                    bond_strength.shear_strength_pa,
                    bond_strength.bending_strength_pa,
                    bond_strength.torsional_strength_pa,
                ]
                .iter()
                .all(|value| value.is_finite() && *value > 0.0),
                "component '{id}' bond strength values must be positive and finite"
            );
        }
        if let Some(destructibility) = &definition.destructibility {
            ensure!(
                matches!(&definition.collider, ColliderShape::Cuboid { .. }),
                "destructible component definition '{id}' must use a cuboid collider"
            );
            ensure!(
                destructibility.impact_speed_threshold_m_s.is_finite()
                    && destructibility.impact_speed_threshold_m_s > 0.0,
                "component definition '{id}' destruction threshold must be positive and finite"
            );
            ensure!(
                destructibility
                    .fragments_per_axis
                    .iter()
                    .all(|count| (2..=4).contains(count))
                    && destructibility.fragments_per_axis.iter().product::<u32>() <= 64,
                "component definition '{id}' fragments_per_axis must be in [2, 4] with at most 64 fragments"
            );
            ensure!(
                definition.visual.model.is_none(),
                "destructible component definition '{id}' must use a procedural primitive visual"
            );
        }
    }

    ensure!(!world.id.trim().is_empty(), "world id cannot be empty");
    ensure!(!world.name.trim().is_empty(), "world name cannot be empty");
    if let Some(gravity) = world.gravity_m_s2 {
        ensure!(
            gravity.iter().all(|value| value.is_finite()),
            "world gravity override must contain only finite values"
        );
    }

    let mut ids = HashSet::with_capacity(world.objects.len());
    for instance in &world.objects {
        ensure!(
            !instance.id.trim().is_empty(),
            "instance ids cannot be empty"
        );
        ensure!(
            ids.insert(instance.id.as_str()),
            "duplicate world instance id '{}'",
            instance.id
        );
        ensure!(
            components.definitions.contains_key(&instance.definition),
            "world instance '{}' references unknown component definition '{}'",
            instance.id,
            instance.definition
        );
        ensure!(
            instance.position_m.iter().all(|value| value.is_finite()),
            "world instance '{}' position must contain only finite values",
            instance.id
        );
        ensure!(
            instance
                .rotation_degrees_xyz
                .iter()
                .all(|value| value.is_finite()),
            "world instance '{}' rotation must contain only finite values",
            instance.id
        );
    }

    let mut chunk_ids = HashSet::new();
    let mut chunk_coordinates = HashSet::new();
    for chunk in chunks {
        ensure!(!chunk.id.trim().is_empty(), "chunk id cannot be empty");
        ensure!(
            chunk_ids.insert(chunk.id.as_str()),
            "duplicate chunk id '{}'",
            chunk.id
        );
        ensure!(
            chunk_coordinates.insert(chunk.coordinate),
            "duplicate chunk coordinate {:?}",
            chunk.coordinate
        );
        ensure!(
            chunk
                .dimensions
                .iter()
                .all(|dimension| (1..=32).contains(dimension)),
            "chunk '{}' dimensions must each be in [1, 32]",
            chunk.id
        );
        ensure!(
            chunk.voxel_size_m.is_finite() && chunk.voxel_size_m > 0.0,
            "chunk '{}' voxel_size_m must be positive and finite",
            chunk.id
        );
    }

    Ok(())
}

/// Validates a chunk payload after its asynchronous read completes.
pub fn validate_chunk_file(
    chunk: &ChunkFile,
    source: &ChunkSource,
    components: &ComponentCatalog,
) -> Result<()> {
    ensure!(
        chunk.schema_version == SCHEMA_VERSION,
        "unsupported schema version {} in chunk '{}'",
        chunk.schema_version,
        chunk.id
    );
    ensure!(
        chunk.id == source.id,
        "chunk id '{}' does not match region index id '{}'",
        chunk.id,
        source.id
    );
    ensure!(
        chunk.coordinate == source.coordinate,
        "chunk '{}' coordinates do not match its region index",
        chunk.id
    );
    ensure!(
        chunk.dimensions == source.dimensions,
        "chunk '{}' dimensions do not match its region index",
        chunk.id
    );
    ensure!(
        (chunk.voxel_size_m - source.voxel_size_m).abs()
            <= f32::EPSILON * source.voxel_size_m.max(1.0),
        "chunk '{}' voxel size does not match its region index",
        chunk.id
    );
    ensure!(
        chunk
            .dimensions
            .iter()
            .all(|dimension| (1..=32).contains(dimension)),
        "chunk '{}' dimensions must each be in [1, 32]",
        chunk.id
    );
    ensure!(
        chunk.voxel_size_m.is_finite() && chunk.voxel_size_m > 0.0,
        "chunk '{}' voxel_size_m must be positive and finite",
        chunk.id
    );
    ensure!(
        !chunk.palette.is_empty() && chunk.palette.len() <= 16 && chunk.palette[0] == "air",
        "chunk '{}' palette must contain 1-16 entries and reserve index zero for 'air'",
        chunk.id
    );

    for (index, definition_id) in chunk.palette.iter().enumerate().skip(1) {
        let Some(definition) = components.definitions.get(definition_id) else {
            anyhow::bail!(
                "chunk '{}' palette entry {index} references unknown definition '{definition_id}'",
                chunk.id
            );
        };
        ensure!(
            matches!(&definition.collider, ColliderShape::Cuboid { .. }),
            "chunk '{}' palette entry '{definition_id}' must use a cuboid collider",
            chunk.id
        );
    }

    let mut layer_ids = HashSet::new();
    for layer in &chunk.layers {
        ensure!(
            layer.y < chunk.dimensions[1],
            "chunk '{}' layer y={} is outside its height",
            chunk.id,
            layer.y
        );
        ensure!(
            layer_ids.insert(layer.y),
            "chunk '{}' has duplicate layer y={}",
            chunk.id,
            layer.y
        );
        ensure!(
            layer.rows.len() == chunk.dimensions[2] as usize,
            "chunk '{}' layer {} must contain exactly {} rows",
            chunk.id,
            layer.y,
            chunk.dimensions[2]
        );
        for (z, row) in layer.rows.iter().enumerate() {
            ensure!(
                row.len() == chunk.dimensions[0] as usize && row.is_ascii(),
                "chunk '{}' layer {} row {} must contain exactly {} ASCII palette characters",
                chunk.id,
                layer.y,
                z,
                chunk.dimensions[0]
            );
            for (x, character) in row.chars().enumerate() {
                let Some(palette_index) = character.to_digit(16) else {
                    anyhow::bail!(
                        "chunk '{}' layer {} row {} column {} has invalid palette character '{}'",
                        chunk.id,
                        layer.y,
                        z,
                        x,
                        character
                    );
                };
                ensure!(
                    palette_index < chunk.palette.len() as u32,
                    "chunk '{}' references palette index {} outside its palette",
                    chunk.id,
                    palette_index
                );
            }
        }
    }

    Ok(())
}

fn validate_collider(collider: &ColliderShape, owner: &str) -> Result<()> {
    match collider {
        ColliderShape::Cuboid { half_extents } => ensure!(
            half_extents
                .iter()
                .all(|value| value.is_finite() && *value > 0.0),
            "component '{owner}' cuboid half_extents must be positive and finite"
        ),
        ColliderShape::Sphere { radius } => ensure!(
            radius.is_finite() && *radius > 0.0,
            "component '{owner}' sphere radius must be positive and finite"
        ),
        ColliderShape::Capsule {
            radius,
            segment_length,
        } => ensure!(
            radius.is_finite()
                && *radius > 0.0
                && segment_length.is_finite()
                && *segment_length >= 0.0,
            "component '{owner}' capsule dimensions are invalid"
        ),
    }
    Ok(())
}

fn validate_visual(visual: &super::schema::VisualDefinition, owner: &str) -> Result<()> {
    ensure!(
        visual
            .color_srgba
            .iter()
            .all(|value| value.is_finite() && (0.0..=1.0).contains(value)),
        "component '{owner}' visual color must be finite sRGBA values in [0, 1]"
    );
    ensure!(
        visual.perceptual_roughness.is_finite()
            && (0.0..=1.0).contains(&visual.perceptual_roughness),
        "component '{owner}' roughness must be in [0, 1]"
    );
    ensure!(
        visual.metallic.is_finite() && (0.0..=1.0).contains(&visual.metallic),
        "component '{owner}' metallic value must be in [0, 1]"
    );
    if matches!(visual.alpha_mode, AlphaMode::Opaque) {
        ensure!(
            (visual.color_srgba[3] - 1.0).abs() <= f32::EPSILON,
            "component '{owner}' opaque visual alpha must equal 1.0"
        );
    }

    if let Some(model) = &visual.model {
        validate_asset_path(model).map_err(|error| {
            anyhow::anyhow!("component '{owner}' has invalid model path: {error}")
        })?;
    }
    Ok(())
}

fn validate_asset_path(asset: &str) -> Result<()> {
    let mut parts = asset.split('#');
    let asset_path = parts.next().unwrap_or_default();
    let scene_selector = parts.next();
    ensure!(
        parts.next().is_none(),
        "model reference may contain only one scene selector"
    );
    ensure!(!asset_path.trim().is_empty(), "model path is empty");
    ensure!(
        !asset_path.starts_with('/') && !asset_path.contains('\\'),
        "model paths must be relative and use forward slashes"
    );
    if let Some(selector) = scene_selector {
        let scene_index = selector
            .strip_prefix("Scene")
            .ok_or_else(|| anyhow::anyhow!("model scene selector must use the form '#SceneN'"))?;
        ensure!(
            !scene_index.is_empty() && scene_index.bytes().all(|byte| byte.is_ascii_digit()),
            "model scene selector must use the form '#SceneN'"
        );
        ensure!(
            scene_index.parse::<usize>().is_ok(),
            "model scene selector index is outside the supported range"
        );
    }
    for component in asset_path.split('/') {
        ensure!(
            !component.is_empty() && component != "." && component != "..",
            "model paths cannot contain empty, current-directory, or parent-directory components"
        );
        ensure!(
            !component.contains(':'),
            "model paths cannot contain platform drive or URI prefixes"
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use crate::config::RuntimeConfig;

    #[test]
    fn collider_validation_accepts_valid_shapes_and_rejects_invalid_dimensions() {
        use super::validate_collider;
        use crate::config::ColliderShape;

        assert!(
            validate_collider(
                &ColliderShape::Cuboid {
                    half_extents: [0.5, 0.5, 0.5]
                },
                "valid-cuboid"
            )
            .is_ok()
        );
        assert!(validate_collider(&ColliderShape::Sphere { radius: 0.5 }, "valid-sphere").is_ok());
        assert!(
            validate_collider(
                &ColliderShape::Capsule {
                    radius: 0.25,
                    segment_length: 0.5
                },
                "valid-capsule"
            )
            .is_ok()
        );
        assert!(
            validate_collider(&ColliderShape::Sphere { radius: 0.0 }, "invalid-sphere").is_err()
        );
    }

    #[test]
    fn model_scene_selectors_are_explicit_and_numeric() {
        use super::validate_asset_path;

        assert!(validate_asset_path("models/part.glb").is_ok());
        assert!(validate_asset_path("models/part.glb#Scene0").is_ok());
        assert!(validate_asset_path("models/part.glb#Scene12").is_ok());
        assert!(validate_asset_path("models/part.glb#Scene").is_err());
        assert!(validate_asset_path("models/part.glb#DefaultScene").is_err());
        assert!(validate_asset_path("models/part.glb#Scene-1").is_err());
    }

    #[test]
    fn checked_in_data_and_world_are_valid_without_reading_user_save_data() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let config = RuntimeConfig::load(
            root.join("data"),
            root.join("assets"),
            root.join("worlds/sandbox/world.json"),
        )
        .expect("sample configuration should validate");

        assert_eq!(config.world.id, "sandbox");
        assert_eq!(config.world.objects.len(), 4);
        assert_eq!(config.chunks.len(), 4);
        assert_eq!(
            config.world.objects[1].definition,
            config.world.objects[2].definition
        );
    }
}

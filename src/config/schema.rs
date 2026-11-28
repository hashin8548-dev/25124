use std::collections::BTreeMap;

use serde::Deserialize;

/// Maximum edge length supported by the bounded in-memory voxel mesher.
pub(crate) const MAX_CHUNK_DIMENSION: u32 = 32;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EngineSettings {
    pub schema_version: u32,
    pub physics: PhysicsSettings,
    pub camera: CameraSettings,
    pub rendering: RenderingSettings,
    pub streaming: StreamingSettings,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhysicsSettings {
    pub meters_per_unit: f32,
    pub gravity_m_s2: [f32; 3],
    pub fixed_hz: f64,
    pub substeps: u32,
    pub restitution_threshold_m_s: f32,
    pub restitution_iterations: u32,
    pub contact_damping_ratio: f32,
    pub contact_frequency_factor: f32,
    pub max_overlap_solve_speed_m_s: f32,
    pub warm_start_coefficient: f32,
    pub sleep_linear_threshold_m_s: f32,
    pub sleep_angular_threshold_rad_s: f32,
    pub sleep_after_seconds: f32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraSettings {
    pub spawn_position_m: [f32; 3],
    pub yaw_degrees: f32,
    pub pitch_degrees: f32,
    pub move_speed_m_s: f32,
    pub sprint_multiplier: f32,
    pub mouse_radians_per_pixel: f32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RenderingSettings {
    pub key_light_illuminance_lux: f32,
    pub key_light_rotation_degrees_xyz: [f32; 3],
    pub shadows_enabled: bool,
}

fn default_object_chunk_size_m() -> f32 {
    16.0
}

fn default_lod_mid_distance_chunks() -> f32 {
    1.75
}

fn default_lod_visibility_distance_chunks() -> u32 {
    512
}

fn default_lod_geometry_errors_m() -> [f32; 3] {
    [0.75, 0.25, 0.05]
}

fn default_lod_quality() -> u8 {
    1
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StreamingSettings {
    pub load_radius_chunks: u32,
    pub unload_radius_chunks: u32,
    #[serde(default = "default_object_chunk_size_m")]
    pub object_chunk_size_m: f32,
    pub max_parallel_chunk_loads: u32,
    pub max_chunk_spawns_per_frame: u32,
    pub greedy_meshing_enabled: bool,
    pub lod_enabled: bool,
    pub lod_near_distance_chunks: f32,
    #[serde(default = "default_lod_mid_distance_chunks")]
    pub lod_mid_distance_chunks: f32,
    pub lod_far_distance_chunks: f32,
    /// Independent outer radius for render-only hierarchical LOD coverage.
    #[serde(default = "default_lod_visibility_distance_chunks")]
    pub lod_visibility_distance_chunks: u32,
    /// Performance, Balanced, Quality world-space surface error thresholds.
    #[serde(default = "default_lod_geometry_errors_m")]
    pub lod_geometry_errors_m: [f32; 3],
    /// Selected geometry-quality profile: 0 = Performance, 1 = Balanced, 2 = Quality.
    #[serde(default = "default_lod_quality")]
    pub lod_quality: u8,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MaterialCatalog {
    pub schema_version: u32,
    pub materials: BTreeMap<String, MaterialDefinition>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MaterialDefinition {
    pub density_kg_m3: f32,
    pub static_friction: f32,
    pub dynamic_friction: f32,
    pub restitution: f32,
    #[serde(default)]
    pub rolling_resistance_coefficient: f32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentCatalog {
    pub schema_version: u32,
    pub definitions: BTreeMap<String, ComponentDefinition>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentDefinition {
    pub body_type: BodyType,
    pub collider: ColliderShape,
    pub material: String,
    pub swept_ccd: Option<SweptCcdDefinition>,
    #[serde(default)]
    pub destructibility: Option<DestructibilityDefinition>,
    #[serde(default)]
    pub bond_strength: Option<BondStrengthDefinition>,
    pub visual: VisualDefinition,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DestructibilityDefinition {
    pub impact_speed_threshold_m_s: f32,
    pub fragments_per_axis: [u32; 3],
}

/// Material-level stiffness, progressive damage and section failure limits for structural bonds.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BondStrengthDefinition {
    #[serde(default = "default_elastic_modulus_pa")]
    pub elastic_modulus_pa: f32,
    pub tensile_strength_pa: f32,
    #[serde(default = "default_compressive_strength_pa")]
    pub compressive_strength_pa: f32,
    pub shear_strength_pa: f32,
    pub bending_strength_pa: f32,
    pub torsional_strength_pa: f32,
    #[serde(default = "default_damage_rate_per_s")]
    pub damage_rate_per_s: f32,
}

fn default_elastic_modulus_pa() -> f32 {
    1.0e9
}
fn default_compressive_strength_pa() -> f32 {
    1.0e7
}
fn default_damage_rate_per_s() -> f32 {
    1.0
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum CcdFilterDefinition {
    Default,
    Dynamic,
    All,
    None,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum SweepModeDefinition {
    Linear,
    NonLinear,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweptCcdDefinition {
    pub filter: CcdFilterDefinition,
    pub mode: SweepModeDefinition,
    pub threshold: f32,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum BodyType {
    Static,
    Dynamic,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ColliderShape {
    Cuboid { half_extents: [f32; 3] },
    Sphere { radius: f32 },
    Capsule { radius: f32, segment_length: f32 },
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum AlphaMode {
    Opaque,
    Blend,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VisualDefinition {
    pub model: Option<String>,
    pub color_srgba: [f32; 4],
    pub perceptual_roughness: f32,
    pub metallic: f32,
    pub alpha_mode: AlphaMode,
}

/// Declarative world-generation parameters. Generated terrain is materialized only for
/// chunks requested by the streaming scheduler.
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum WorldGeneration {
    FlatPlatform {
        size_units: u64,
        border_height_units: u32,
        border_thickness_units: u32,
    },
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorldManifest {
    pub schema_version: u32,
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub preview_asset: Option<String>,
    #[serde(default)]
    pub metadata: BTreeMap<String, serde_json::Value>,
    pub gravity_m_s2: Option<[f32; 3]>,
    /// Optional deterministic terrain recipe; region files remain supported for authored worlds.
    #[serde(default)]
    pub generation: Option<WorldGeneration>,
    #[serde(default)]
    pub regions: Vec<String>,
    pub objects: Vec<WorldObjectInstance>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegionManifest {
    pub schema_version: u32,
    pub id: String,
    pub coordinate: [i32; 2],
    pub chunks: Vec<ChunkReference>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChunkReference {
    pub id: String,
    pub coordinate: [i32; 3],
    pub dimensions: [u32; 3],
    pub voxel_size_m: f32,
    pub path: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChunkFile {
    pub schema_version: u32,
    pub id: String,
    pub coordinate: [i32; 3],
    pub dimensions: [u32; 3],
    pub voxel_size_m: f32,
    /// Palette index zero is reserved for air. Remaining entries reference component definitions.
    pub palette: Vec<String>,
    /// Each row contains one hexadecimal palette index per X cell; rows are ordered by Z.
    pub layers: Vec<ChunkLayer>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChunkLayer {
    pub y: u32,
    pub rows: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorldObjectInstance {
    pub id: String,
    pub definition: String,
    pub position_m: [f32; 3],
    pub rotation_degrees_xyz: [f32; 3],
}

#[cfg(test)]
mod tests {
    use super::StreamingSettings;

    #[test]
    fn legacy_streaming_settings_default_new_spatial_and_lod_fields() {
        let settings: StreamingSettings = serde_json::from_str(
            r#"{
                "load_radius_chunks": 2,
                "unload_radius_chunks": 3,
                "max_parallel_chunk_loads": 2,
                "max_chunk_spawns_per_frame": 2,
                "greedy_meshing_enabled": true,
                "lod_enabled": true,
                "lod_near_distance_chunks": 1.0,
                "lod_far_distance_chunks": 2.5
            }"#,
        )
        .unwrap();

        assert_eq!(settings.object_chunk_size_m, 16.0);
        assert_eq!(settings.lod_mid_distance_chunks, 1.75);
        assert_eq!(settings.lod_visibility_distance_chunks, 512);
        assert_eq!(settings.lod_geometry_errors_m, [0.75, 0.25, 0.05]);
        assert_eq!(settings.lod_quality, 1);
    }
}

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

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StreamingSettings {
    pub load_radius_chunks: u32,
    pub unload_radius_chunks: u32,
    pub max_parallel_chunk_loads: u32,
    pub max_chunk_spawns_per_frame: u32,
    pub greedy_meshing_enabled: bool,
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

/// Material-level failure thresholds for voxel-to-voxel structural bonds.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BondStrengthDefinition {
    pub tensile_strength_pa: f32,
    pub shear_strength_pa: f32,
    pub bending_strength_pa: f32,
    pub torsional_strength_pa: f32,
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

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorldManifest {
    pub schema_version: u32,
    pub id: String,
    pub name: String,
    pub gravity_m_s2: Option<[f32; 3]>,
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

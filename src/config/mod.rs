mod library;
mod load;
mod schema;
mod validate;

pub use library::WorldLibrary;
pub(crate) use load::object_chunk_coordinate;
pub use load::{ChunkSource, ObjectChunkSource, RuntimeConfig};
pub use schema::{
    AlphaMode, BodyType, BondStrengthDefinition, CcdFilterDefinition, ChunkFile, ChunkLayer,
    ColliderShape, ComponentDefinition, DestructibilityDefinition, MaterialDefinition,
    SweepModeDefinition, VisualDefinition, WorldGeneration,
};
pub(crate) use schema::{ComponentCatalog, MAX_CHUNK_DIMENSION};
pub(crate) use validate::validate_chunk_file;

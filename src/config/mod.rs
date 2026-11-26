mod load;
mod schema;
mod validate;

pub(crate) use schema::MAX_CHUNK_DIMENSION;
pub use load::{ChunkSource, RuntimeConfig};
pub use schema::{
    AlphaMode, BodyType, BondStrengthDefinition, CcdFilterDefinition, ChunkFile, ColliderShape,
    ComponentDefinition, DestructibilityDefinition, MaterialDefinition, SweepModeDefinition,
    VisualDefinition,
};
pub(crate) use validate::validate_chunk_file;

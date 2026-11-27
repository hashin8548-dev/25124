mod generation;\nmod greedy_mesher;
mod ids;
mod lod;
mod spawn;
mod streaming;

pub(crate) use generation::WorldOrigin;\npub(crate) use ids::{parse_voxel_instance_id, voxel_instance_id};
pub(crate) use spawn::SpawnedVoxelBody;

use bevy::prelude::*;

#[derive(SystemSet, Clone, Debug, PartialEq, Eq, Hash)]
pub(crate) struct ChunkStreamingUpdateSet;

#[derive(Component, Clone, Debug, PartialEq, Eq)]
pub(crate) struct ChunkMembership(pub String);

pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<streaming::ChunkStreamingState>()
            .add_systems(
                Update,
                streaming::update_chunk_streaming
                    .after(crate::physics::ContactUpdateSet)\n                    .after(generation::rebase_world_origin)
                    .in_set(ChunkStreamingUpdateSet),
            )
            .add_systems(Update, lod::update_chunk_lod.after(ChunkStreamingUpdateSet));
    }
}

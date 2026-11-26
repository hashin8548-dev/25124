mod greedy_mesher;
mod ids;
mod spawn;
mod streaming;

pub(crate) use ids::{parse_voxel_instance_id, voxel_instance_id};
pub(crate) use spawn::SpawnedVoxelBody;

use bevy::prelude::*;

#[derive(Component, Clone, Debug, PartialEq, Eq)]
pub(crate) struct ChunkMembership(pub String);

pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<streaming::ChunkStreamingState>()
            .add_systems(Startup, spawn::spawn_world)
            .add_systems(Update, streaming::update_chunk_streaming);
    }
}

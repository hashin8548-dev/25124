mod ids;
mod spawn;

pub(crate) use ids::voxel_instance_id;

use bevy::prelude::*;

pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn::spawn_world);
    }
}

mod motion;

pub use motion::{HumanAction, HumanMotionLibrary, MotionTrack};

use bevy::prelude::*;
use crate::config::RuntimeConfig;

/// Registers the external human action data source for the simulation runtime.
pub struct HumanPlugin;

impl Plugin for HumanPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, load_motion_library);
    }
}

fn load_motion_library(mut commands: Commands, config: Res<RuntimeConfig>) {
    match HumanMotionLibrary::load(&config.paths.data_dir) {
        Ok(library) => {
            info!(
                "Loaded human motion actions: {}",
                library.action_names().collect::<Vec<_>>().join(", ")
            );
            commands.insert_resource(library);
        }
        Err(error) => {
            warn!("Human motion data is unavailable; human gameplay systems remain inactive: {error:#}");
        }
    }
}

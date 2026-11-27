mod motion;
mod rig;

pub use motion::{HumanAction, HumanMotionLibrary, MotionTrack};
pub use rig::{BoneCollisionGeometryValue, HumanBonePose, HumanRig, RigBone};

use crate::config::RuntimeConfig;
use bevy::prelude::*;

pub struct HumanPlugin;

impl Plugin for HumanPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, load_human_data);
    }
}

fn load_human_data(mut commands: Commands, config: Res<RuntimeConfig>) {
    match HumanMotionLibrary::load(&config.paths.data_dir) {
        Ok(library) => {
            info!(
                "Loaded human motion actions: {}",
                library.action_names().collect::<Vec<_>>().join(", ")
            );
            commands.insert_resource(library);
        }
        Err(error) => {
            warn!(
                "Human motion data is unavailable; human gameplay systems remain inactive: {error:#}"
            );
            return;
        }
    }

    match HumanRig::load(&config.paths.data_dir) {
        Ok(rig) => {
            info!(
                "Loaded human deformation rig: {} body tracks and {} joint markers",
                rig.body_names().count(),
                rig.marker_names().count()
            );
            commands.insert_resource(rig);
        }
        Err(error) => {
            warn!(
                "Human rig metadata is unavailable; human ragdoll systems remain inactive: {error:#}"
            );
        }
    }
}

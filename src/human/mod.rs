mod actuation;
mod input;
mod motion;
mod ragdoll;
mod rig;
mod skinning;

pub use actuation::HumanActionState;
pub use motion::{HumanAction, HumanMotionLibrary, MotionTrack};
pub use ragdoll::{
    HumanBodyPart, HumanCameraSettings, HumanControlSettings, HumanJointLimits,
    HumanPhysicsSettings, HumanPlayer, HumanPlayerRoot, HumanSkinnedSceneRoot,
    HumanTemporaryAssists, JumpLaunchAssist, RotationalAssist,
};
pub use rig::{
    BoneCollisionGeometry, BoneCollisionGeometryValue, HumanBonePose, HumanRig, RigBone,
};

use crate::{config::RuntimeConfig, engine::AppMode};
use bevy::prelude::*;

/// Integrates a physics-authoritative human ragdoll into the simulation.
/// Motion JSON supplies pose targets; dynamic bodies, contacts and joints determine motion.
pub struct HumanPlugin;

impl Plugin for HumanPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, load_human_data)
            .add_systems(OnEnter(AppMode::Simulation), ragdoll::spawn_human_player)
            .add_systems(OnExit(AppMode::Simulation), ragdoll::despawn_human_player)
            .add_systems(
                FixedPostUpdate,
                actuation::drive_human_action_pose
                    .in_set(avian3d::schedule::PhysicsSystems::First)
                    .run_if(in_state(AppMode::Simulation)),
            )
            .add_systems(
                Update,
                input::update_human_controls.run_if(in_state(AppMode::Simulation)),
            )
            .add_systems(
                PostUpdate,
                skinning::sync_human_skinned_mesh
                    .after(TransformSystems::Propagate)
                    .run_if(in_state(AppMode::Simulation)),
            );
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

    match HumanPhysicsSettings::load(&config.paths.data_dir) {
        Ok(settings) => {
            info!("Loaded human physical material and joint settings");
            commands.insert_resource(settings);
        }
        Err(error) => {
            warn!("Human physics settings are invalid; human ragdoll remains inactive: {error:#}");
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

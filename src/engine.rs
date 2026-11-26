use avian3d::{
    dynamics::solver::SolverConfig,
    prelude::{Gravity, PhysicsPlugins, SubstepCount},
};
use bevy::prelude::*;

use crate::{
    camera::CameraPlugin,
    config::RuntimeConfig,
    persistence::{PersistencePlugin, WorldSaveTarget},
    physics::ContactPlugin,
    rendering::RenderingPlugin,
    ui::MenuPlugin,
    world::WorldPlugin,
};

pub struct EnginePlugin;

impl Plugin for EnginePlugin {
    fn build(&self, app: &mut App) {
        let config = app.world().resource::<RuntimeConfig>().clone();
        let physics = &config.engine.physics;
        let gravity = config
            .world
            .gravity_m_s2
            .unwrap_or(physics.gravity_m_s2);

        app.add_plugins(PhysicsPlugins::default())
            .insert_resource(Time::<Fixed>::from_hz(physics.fixed_hz))
            .insert_resource(SubstepCount(physics.substeps))
            .insert_resource(Gravity(Vec3::from_array(gravity)))
            .insert_resource(SolverConfig {
                restitution_threshold: physics.restitution_threshold_m_s,
                restitution_iterations: physics.restitution_iterations as usize,
                contact_damping_ratio: physics.contact_damping_ratio,
                contact_frequency_factor: physics.contact_frequency_factor,
                max_overlap_solve_speed: physics.max_overlap_solve_speed_m_s,
                warm_start_coefficient: physics.warm_start_coefficient,
            })
            .insert_resource(WorldSaveTarget {
                world_id: config.world.id.clone(),
                state_path: config.paths.world_state.clone(),
            })
            .add_plugins((
                WorldPlugin,
                RenderingPlugin,
                CameraPlugin,
                ContactPlugin,
                PersistencePlugin,
                MenuPlugin,
            ));
    }
}

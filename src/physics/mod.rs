mod bonds;
#[cfg(test)]
mod collision_validation;
mod contacts;
mod coupled_restitution;
mod destructibility;
mod rolling;
mod structural;

pub use bonds::{VoxelBond, VoxelBondSpec, spawn_voxel_bond};
pub use contacts::ContactLedger;
pub use destructibility::{DestructibleBody, DestructibleBodySpec, DestructionJournal};
pub use rolling::RollingResistance;

use avian3d::{
    dynamics::solver::schedule::SolverSystems,
    prelude::{PhysicsSchedule, PhysicsStepSystems},
};
use bevy::prelude::*;

#[derive(SystemSet, Clone, Debug, PartialEq, Eq, Hash)]
pub(crate) struct ContactUpdateSet;

pub struct ContactPlugin;

impl Plugin for ContactPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ContactLedger>()
            .init_resource::<DestructionJournal>()
            .add_systems(Startup, destructibility::restore_destruction_journal)
            // Contacts are refreshed in NarrowPhase. Apply the custom angular impulse after
            // that refresh, but before Solver consumes the manifolds for this physics step.
            // Putting this system in PhysicsStepSystems::First races Avian's previous-velocity
            // capture because both systems mutate AngularVelocity.
            .add_systems(
                PhysicsSchedule,
                rolling::apply_contact_rolling_resistance
                    .after(PhysicsStepSystems::NarrowPhase)
                    .before(PhysicsStepSystems::Solver),
            )
            // Resolve restitution constraints as one coupled contact graph. Avian 0.8's
            // built-in restitution pass operates per constraint, so shared-body contacts
            // otherwise lose separation speed when their impulses undo one another.
            .add_systems(
                PhysicsSchedule,
                coupled_restitution::solve_coupled_restitution
                    .in_set(PhysicsStepSystems::Solver)
                    .after(SolverSystems::StoreContactImpulses),
            )
            .add_systems(
                Update,
                (
                    contacts::collect_contact_transitions,
                    bonds::break_overstressed_bonds,
                    destructibility::fracture_on_new_impacts,
                )
                    .chain()
                    .in_set(ContactUpdateSet),
            );
    }
}

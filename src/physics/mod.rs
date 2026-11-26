mod bonds;
mod contacts;
mod destructibility;
mod rolling;

pub use bonds::{VoxelBond, spawn_voxel_bond};
pub use contacts::ContactLedger;
pub use destructibility::{DestructibleBody, DestructionJournal};
pub use rolling::RollingResistance;

use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems};
use bevy::prelude::*;

pub struct ContactPlugin;

impl Plugin for ContactPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ContactLedger>()
            .init_resource::<DestructionJournal>()
            .add_systems(Startup, destructibility::restore_destruction_journal)
            .add_systems(
                PhysicsSchedule,
                rolling::apply_contact_rolling_resistance.in_set(PhysicsStepSystems::First),
            )
            .add_systems(
                Update,
                (
                    contacts::collect_contact_transitions,
                    bonds::break_overstressed_bonds,
                    destructibility::fracture_on_new_impacts,
                )
                    .chain(),
            );
    }
}

mod contacts;
mod destructibility;

pub use contacts::ContactLedger;
pub use destructibility::{DestructibleBody, DestructionJournal};

use bevy::prelude::*;

pub struct ContactPlugin;

impl Plugin for ContactPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ContactLedger>()
            .init_resource::<DestructionJournal>()
            .add_systems(
                Startup,
                destructibility::restore_destruction_journal,
            )
            .add_systems(
                Update,
                (
                    contacts::collect_contact_transitions,
                    destructibility::fracture_on_new_impacts,
                )
                    .chain(),
            );
    }
}

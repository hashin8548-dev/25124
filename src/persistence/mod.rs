mod state;

pub use state::{PersistedBodyState, PersistedFragmentState, WorldStateFile};

use std::path::PathBuf;

use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;

use crate::{config::{BodyType, RuntimeConfig}, physics::DestructionJournal};

#[derive(Component, Clone, Debug)]
pub struct PersistentObjectId(pub String);

/// Marks bodies whose runtime transform and velocities override authored world placement.
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct PersistRuntimeState;

/// Marks a generated fragment with the definition and local collider dimensions needed for recovery.
#[derive(Component, Clone, Debug)]
pub struct PersistedFragment {
    pub chunk_id: Option<String>,
    pub definition_id: String,
    pub half_extents_m: [f32; 3],
}

#[derive(Resource, Clone, Debug)]
pub struct WorldSaveTarget {
    pub world_id: String,
    pub state_path: PathBuf,
}

fn known_static_world_object_ids(config: &RuntimeConfig) -> HashSet<String> {
    config
        .world
        .objects
        .iter()
        .filter_map(|instance| {
            config
                .components
                .definitions
                .get(&instance.definition)
                .filter(|definition| matches!(definition.body_type, BodyType::Static))
                .map(|_| instance.id.clone())
        })
        .collect()
}

pub struct PersistencePlugin;

impl Plugin for PersistencePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, save_world_on_f5);
    }
}

fn save_world_on_f5(
    keys: Res<ButtonInput<KeyCode>>,
    target: Res<WorldSaveTarget>,
    config: Res<RuntimeConfig>,
    previous_state: Res<WorldStateFile>,
    destruction_journal: Res<DestructionJournal>,
    bodies: Query<
        (
            &PersistentObjectId,
            &Transform,
            &LinearVelocity,
            &AngularVelocity,
        ),
        With<PersistRuntimeState>,
    >,
    fragments: Query<(
        &PersistentObjectId,
        &PersistedFragment,
        &Transform,
        &LinearVelocity,
        &AngularVelocity,
    )>,
) {
    if !keys.just_pressed(KeyCode::F5) {
        return;
    }

    // Keep state for streamed-out chunks and unavailable mod content. Live bodies and fragments
    // overwrite their records below, while static authored cells remain manifest-owned.
    let mut snapshot = WorldStateFile::new(target.world_id.clone());
    // Keep body records for chunks that are currently streamed out. Live entities below
    // overwrite their own IDs with current runtime state.
    let static_ids = known_static_world_object_ids(&config);
    snapshot.bodies = previous_state
        .bodies
        .iter()
        .filter(|(id, _)| !static_ids.contains(*id))
        .map(|(id, state)| (id.clone(), state.clone()))
        .collect();
    snapshot.fractured = destruction_journal.fractured.clone();
    snapshot.broken_bonds = destruction_journal.broken_bonds.clone();
    snapshot.fragments = previous_state.fragments.clone();

    for (id, transform, linear_velocity, angular_velocity) in &bodies {
        snapshot.bodies.insert(
            id.0.clone(),
            PersistedBodyState {
                translation_m: transform.translation.to_array(),
                rotation_xyzw: transform.rotation.to_array(),
                linear_velocity_m_s: linear_velocity.0.to_array(),
                angular_velocity_rad_s: angular_velocity.0.to_array(),
            },
        );
    }

    for (id, fragment, transform, linear_velocity, angular_velocity) in &fragments {
        snapshot.fragments.insert(
            id.0.clone(),
            PersistedFragmentState {
                chunk_id: fragment.chunk_id.clone(),
                definition_id: fragment.definition_id.clone(),
                half_extents_m: fragment.half_extents_m,
                translation_m: transform.translation.to_array(),
                rotation_xyzw: transform.rotation.to_array(),
                linear_velocity_m_s: linear_velocity.0.to_array(),
                angular_velocity_rad_s: angular_velocity.0.to_array(),
            },
        );
    }

    match snapshot.write_atomic(&target.state_path) {
        Ok(()) => info!(
            "Saved {} dynamic body states, {} fragments and {} damage records to '{}'",
            snapshot.bodies.len(),
            snapshot.fragments.len(),
            snapshot.fractured.len() + snapshot.broken_bonds.len(),
            target.state_path.display()
        ),
        Err(error) => error!(
            "Could not save world state to '{}': {error:#}",
            target.state_path.display()
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::{PersistedBodyState, WorldStateFile};

    #[test]
    fn save_snapshot_can_preserve_dynamic_states_for_unloaded_chunks() {
        let mut previous = WorldStateFile::new("test-world".to_owned());
        previous.bodies.insert(
            "voxel:chunk_a:1:2:3".to_owned(),
            PersistedBodyState {
                translation_m: [8.0, 3.0, -2.0],
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                linear_velocity_m_s: [1.0, 0.0, 0.0],
                angular_velocity_rad_s: [0.0; 3],
            },
        );
        let snapshot = previous.clone();

        assert_eq!(
            snapshot.bodies["voxel:chunk_a:1:2:3"].translation_m,
            [8.0, 3.0, -2.0]
        );
        assert_eq!(
            snapshot.bodies["voxel:chunk_a:1:2:3"].linear_velocity_m_s,
            [1.0, 0.0, 0.0]
        );
    }
}

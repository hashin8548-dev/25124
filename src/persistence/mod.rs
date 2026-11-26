mod state;

pub use state::WorldStateFile;

use std::{
    collections::HashSet,
    path::PathBuf,
};

use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;

use crate::{
    config::RuntimeConfig,
    physics::DestructionJournal,
    persistence::state::{PersistedBodyState, PersistedFragmentState},
};

#[derive(Component, Clone, Debug)]
pub struct PersistentObjectId(pub String);

/// Marks bodies whose runtime transform and velocities override authored world placement.
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct PersistRuntimeState;

/// Marks a generated fragment with the definition and local collider dimensions needed for recovery.
#[derive(Component, Clone, Debug)]
pub struct PersistedFragment {
    pub definition_id: String,
    pub half_extents_m: [f32; 3],
}

#[derive(Resource, Clone, Debug)]
pub struct WorldSaveTarget {
    pub world_id: String,
    pub state_path: PathBuf,
}

pub struct PersistencePlugin;

impl Plugin for PersistencePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, save_world_on_f5);
    }
}

fn known_world_instance_ids(config: &RuntimeConfig) -> HashSet<String> {
    let mut known = config
        .world
        .objects
        .iter()
        .map(|instance| instance.id.clone())
        .collect::<HashSet<_>>();

    for chunk in &config.chunks {
        for layer in &chunk.layers {
            for (z, row) in layer.rows.iter().enumerate() {
                for (x, cell) in row.chars().enumerate() {
                    if cell != '0' {
                        known.insert(format!(
                            "voxel:{}:{}:{}:{}",
                            chunk.id, x, layer.y, z
                        ));
                    }
                }
            }
        }
    }

    known
}

fn unresolved_body_states(
    previous_state: &WorldStateFile,
    known_instance_ids: &HashSet<String>,
) -> std::collections::BTreeMap<String, PersistedBodyState> {
    previous_state
        .bodies
        .iter()
        .filter(|(id, _)| !known_instance_ids.contains(*id))
        .map(|(id, state)| (id.clone(), state.clone()))
        .collect()
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
    fragments: Query<
        (
            &PersistentObjectId,
            &PersistedFragment,
            &Transform,
            &LinearVelocity,
            &AngularVelocity,
        ),
    >,
) {
    if !keys.just_pressed(KeyCode::F5) {
        return;
    }

    let known_ids = known_world_instance_ids(&config);

    // Unknown records are retained so temporarily unavailable mod content can be restored later.
    // Known static bodies are omitted: their authored transforms remain owned by chunk/manifest data.
    let mut snapshot = WorldStateFile::new(target.world_id.clone());
    snapshot.bodies = unresolved_body_states(&previous_state, &known_ids);
    snapshot.fractured = destruction_journal.fractured.clone();
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
            "Saved {} dynamic body states, {} fragments and {} fracture records to '{}'",
            snapshot.bodies.len(),
            snapshot.fragments.len(),
            snapshot.fractured.len(),
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
    use std::collections::HashSet;

    use super::{PersistedBodyState, WorldStateFile, unresolved_body_states};

    #[test]
    fn save_snapshot_preserves_only_records_not_owned_by_current_world_instances() {
        let mut previous = WorldStateFile::new("test-world".to_owned());
        for id in ["dynamic", "static", "unresolved-mod-object"] {
            previous.bodies.insert(
                id.to_owned(),
                PersistedBodyState {
                    translation_m: [1.0, 2.0, 3.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    linear_velocity_m_s: [0.0; 3],
                    angular_velocity_rad_s: [0.0; 3],
                },
            );
        }

        let known_instance_ids = HashSet::from(["dynamic".to_owned(), "static".to_owned()]);
        let unresolved = unresolved_body_states(&previous, &known_instance_ids);

        assert_eq!(unresolved.len(), 1);
        assert!(unresolved.contains_key("unresolved-mod-object"));
        assert!(!unresolved.contains_key("static"));
    }
}

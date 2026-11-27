use std::{collections::BTreeMap, fs, path::{Component, Path, PathBuf}};
use anyhow::{Context, Result, ensure};
use bevy::prelude::{Mat4, Resource};
use serde::Deserialize;

/// Validated external action library. Tracks are body-pose targets, never rendered clips.
#[derive(Clone, Debug, Resource)]
pub struct HumanMotionLibrary { actions: BTreeMap<String, HumanAction> }

#[derive(Clone, Debug)]
pub struct HumanAction {
    pub looping: bool,
    tracks: BTreeMap<String, MotionTrack>,
    pub duration_seconds: f32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotionTrack {
    pub name: String,
    pub time: Vec<f32>,
    pub transform: Vec<[f32; 16]>,
}

#[derive(Debug, Deserialize)]
struct HumanManifest { schema_version: u32, actions: BTreeMap<String, ActionReference> }

#[derive(Debug, Deserialize)]
struct ActionReference {
    file: String,
    #[serde(default, rename = "loop")]
    looping: bool,
}

#[derive(Debug, Deserialize)]
struct ActionDocument { animation: Vec<MotionTrack> }

impl HumanMotionLibrary {
    /// Loads the data-driven human manifest and every referenced action JSON file.
    pub fn load(data_dir: &Path) -> Result<Self> {
        let human_dir = data_dir.join("human");
        let manifest_path = human_dir.join("human.json");
        let manifest_text = fs::read_to_string(&manifest_path)
            .with_context(|| format!("reading human manifest {}", manifest_path.display()))?;
        let manifest: HumanManifest = serde_json::from_str(&manifest_text)
            .with_context(|| format!("parsing human manifest {}", manifest_path.display()))?;
        ensure!(manifest.schema_version == 1, "unsupported human manifest schema version {}", manifest.schema_version);
        ensure!(!manifest.actions.is_empty(), "human manifest has no actions");

        let mut actions = BTreeMap::new();
        for (action_name, reference) in manifest.actions {
            let relative = PathBuf::from(&reference.file);
            ensure!(!relative.is_absolute(), "action '{action_name}' path must be relative");
            ensure!(
                relative.components().all(|part| matches!(part, Component::Normal(_))),
                "action '{action_name}' path contains a parent or current-directory component"
            );
            ensure!(relative.extension().is_some_and(|ext| ext == "json"), "action '{action_name}' must reference JSON");
            let action_path = human_dir.join(relative);
            let source = fs::read_to_string(&action_path)
                .with_context(|| format!("reading human action '{action_name}' at {}", action_path.display()))?;
            let document: ActionDocument = serde_json::from_str(&source)
                .with_context(|| format!("parsing human action '{action_name}' at {}", action_path.display()))?;
            let action = HumanAction::from_document(&action_name, document, reference.looping)?;
            ensure!(actions.insert(action_name.clone(), action).is_none(), "duplicate human action name '{action_name}'");
        }
        Ok(Self { actions })
    }

    pub fn action(&self, name: &str) -> Option<&HumanAction> { self.actions.get(name) }
    pub fn action_names(&self) -> impl Iterator<Item = &str> { self.actions.keys().map(String::as_str) }
}

impl HumanAction {
    fn from_document(name: &str, document: ActionDocument, looping: bool) -> Result<Self> {
        ensure!(!document.animation.is_empty(), "human action '{name}' has no tracks");
        let mut tracks = BTreeMap::new();
        let mut duration_seconds: f32 = 0.0;
        for track in document.animation {
            ensure!(!track.name.trim().is_empty(), "human action '{name}' has an empty track name");
            ensure!(!track.time.is_empty(), "human action '{name}' track '{}' has no timestamps", track.name);
            ensure!(
                track.time.len() == track.transform.len(),
                "human action '{name}' track '{}' has {} timestamps but {} transforms",
                track.name, track.time.len(), track.transform.len()
            );
            ensure!(track.time.iter().all(|t| t.is_finite() && *t >= 0.0), "human action '{name}' track '{}' has invalid timestamps", track.name);
            ensure!(track.time.windows(2).all(|pair| pair[1] > pair[0]), "human action '{name}' track '{}' timestamps are not strictly increasing", track.name);
            ensure!(track.transform.iter().flatten().all(|v| v.is_finite()), "human action '{name}' track '{}' has a non-finite matrix value", track.name);
            duration_seconds = duration_seconds.max(*track.time.last().expect("non-empty timestamps"));
            let track_name = track.name.clone();
            ensure!(tracks.insert(track_name.clone(), track).is_none(), "human action '{name}' has duplicate track '{track_name}'");
        }
        ensure!(tracks.contains_key("Root"), "human action '{name}' has no Root track");
        ensure!(duration_seconds > 0.0, "human action '{name}' has zero duration");
        Ok(Self { looping, tracks, duration_seconds })
    }

    pub fn track(&self, name: &str) -> Option<&MotionTrack> { self.tracks.get(name) }
    pub fn track_names(&self) -> impl Iterator<Item = &str> { self.tracks.keys().map(String::as_str) }
}

impl MotionTrack {
    /// Samples a row-major source-space transform at elapsed seconds.
    /// reverse=true samples the same motion in reverse; looping is local to this action.
    pub fn sample_matrix(&self, elapsed_seconds: f32, looping: bool, reverse: bool) -> Mat4 {
        let first = self.time[0];
        let last = *self.time.last().expect("validated non-empty track");
        let duration = last - first;
        if duration <= f32::EPSILON || self.time.len() == 1 {
            return row_major_to_mat4(&self.transform[0]);
        }
        let elapsed = if elapsed_seconds.is_finite() { elapsed_seconds.max(0.0) } else { 0.0 };
        let offset = if looping { elapsed.rem_euclid(duration) } else { elapsed.min(duration) };
        let sample_time = if reverse { last - offset } else { first + offset };
        let upper = self.time.partition_point(|timestamp| *timestamp < sample_time);
        if upper == 0 { return row_major_to_mat4(&self.transform[0]); }
        if upper >= self.time.len() {
            return row_major_to_mat4(self.transform.last().expect("validated non-empty track"));
        }
        let lower = upper - 1;
        let alpha = ((sample_time - self.time[lower]) / (self.time[upper] - self.time[lower])).clamp(0.0, 1.0);
        interpolate_rigid_transform(
            row_major_to_mat4(&self.transform[lower]),
            row_major_to_mat4(&self.transform[upper]),
            alpha,
        )
    }
}

/// JSON stores matrices row-major; Bevy/glam Mat4::from_cols_array is column-major.
fn row_major_to_mat4(values: &[f32; 16]) -> Mat4 { Mat4::from_cols_array(values).transpose() }

/// Interpolate rigid pose components separately to preserve an orthogonal rotation basis.
fn interpolate_rigid_transform(a: Mat4, b: Mat4, alpha: f32) -> Mat4 {
    let (scale_a, rotation_a, translation_a) = a.to_scale_rotation_translation();
    let (scale_b, rotation_b, translation_b) = b.to_scale_rotation_translation();
    Mat4::from_scale_rotation_translation(
        scale_a.lerp(scale_b, alpha),
        rotation_a.slerp(rotation_b, alpha).normalize(),
        translation_a.lerp(translation_b, alpha),
    )
}

#[cfg(test)]
mod tests {
    use super::{HumanAction, MotionTrack};
    use bevy::prelude::Vec3;

    const ACTIONS: [(&str, &str); 8] = [
        ("walk", include_str!("../../data/human/actions/walk.json")),
        ("run", include_str!("../../data/human/actions/run.json")),
        ("crouch", include_str!("../../data/human/actions/sneak.json")),
        ("climb", include_str!("../../data/human/actions/climb.json")),
        ("fall", include_str!("../../data/human/actions/fall.json")),
        ("idle", include_str!("../../data/human/actions/idle.json")),
        ("jump", include_str!("../../data/human/actions/jump.json")),
        ("crouched_idle", include_str!("../../data/human/actions/kneel.json")),
    ];

    #[test]
    fn every_supplied_action_has_valid_body_tracks() {
        for (name, source) in ACTIONS {
            let document: super::ActionDocument = serde_json::from_str(source).unwrap();
            let action = HumanAction::from_document(name, document, true).unwrap();
            assert_eq!(action.track_names().count(), 20, "{name}");
            assert!(action.track("Root").is_some(), "{name}");
            assert!(action.duration_seconds > 0.0, "{name}");
        }
    }

    #[test]
    fn tracks_keep_independent_timestamps() {
        let document: super::ActionDocument = serde_json::from_str(ACTIONS[0].1).unwrap();
        let action = HumanAction::from_document("walk", document, true).unwrap();
        assert_ne!(action.track("Shoulder_R").unwrap().time.len(), action.track("Root").unwrap().time.len());
    }

    #[test]
    fn row_major_translation_is_read_from_last_column() {
        let values = [
            1.0, 0.0, 0.0, 2.0, 0.0, 1.0, 0.0, 3.0,
            0.0, 0.0, 1.0, 4.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let (_, _, translation) = super::row_major_to_mat4(&values).to_scale_rotation_translation();
        assert_eq!(translation, Vec3::new(2.0, 3.0, 4.0));
    }

    #[test]
    fn reverse_sampling_reverses_time_without_mutating_tracks() {
        let track = MotionTrack {
            name: "Root".to_owned(),
            time: vec![0.0, 1.0, 2.0],
            transform: vec![
                [1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0],
                [1.0,0.0,0.0,1.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0],
                [1.0,0.0,0.0,2.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0],
            ],
        };
        let forward = track.sample_matrix(0.5, false, false).w_axis.truncate();
        let reverse = track.sample_matrix(0.5, false, true).w_axis.truncate();
        assert!((forward.x - 0.5).abs() < 1.0e-5);
        assert!((reverse.x - 1.5).abs() < 1.0e-5);
        assert_eq!(track.time, vec![0.0, 1.0, 2.0]);
    }

    #[test]
    fn non_looping_samples_clamp_to_track_end() {
        let document: super::ActionDocument = serde_json::from_str(ACTIONS[0].1).unwrap();
        let action = HumanAction::from_document("walk", document, false).unwrap();
        let root = action.track("Root").unwrap();
        assert_eq!(root.sample_matrix(10_000.0, false, false), root.sample_matrix(action.duration_seconds, false, false));
    }

    #[test]
    fn sampled_matrices_remain_finite_and_rotation_is_normalized() {
        let document: super::ActionDocument = serde_json::from_str(ACTIONS[0].1).unwrap();
        let action = HumanAction::from_document("walk", document, true).unwrap();
        for track in action.tracks.values() {
            let matrix = track.sample_matrix(0.25, true, false);
            assert!(matrix.to_cols_array().iter().all(|value| value.is_finite()));
            let (_, rotation, _) = matrix.to_scale_rotation_translation();
            assert!((rotation.length() - 1.0).abs() < 1.0e-3);
        }
    }
}

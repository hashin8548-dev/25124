use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::Path,
};

use anyhow::{Context, Result, ensure};
use bevy::prelude::{Mat4, Quat, Resource, Vec3};
use serde::Deserialize;

use super::HumanAction;

/// FBX-derived rest hierarchy and mesh-weighted collision geometry for the human entity.
#[derive(Clone, Debug, Resource)]
pub struct HumanRig {
    armature_world: Mat4,
    bones: BTreeMap<String, RigBone>,
    body_names: BTreeSet<String>,
    marker_names: BTreeSet<String>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct RigBone {
    pub name: String,
    pub parent: Option<String>,
    pub deform: bool,
    pub length_m: f32,
    pub matrix_parent_local: [[f32; 4]; 4],
    pub action_reference_parent_local: Option<[f32; 16]>,
    pub collision_geometry: Option<BoneCollisionGeometry>,
    /// Joint-marker bone used as the physical anchor for this body segment.
    pub joint_anchor_track: Option<String>,
    /// Self-identification for bones included in the joint-marker tracks.
    pub joint_marker_track: Option<String>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct BoneCollisionGeometry {
    pub shape: String,
    pub radius_m: f32,
    pub axial_min_m: f32,
    pub axial_max_m: f32,
    pub axial_length_m: f32,
    pub capsule_segment_length_m: f32,
    pub center_offset_bone_m: [f32; 3],
    pub weighted_vertex_count: usize,
    pub weight_threshold: f32,
}

#[derive(Debug, Deserialize)]
struct RigDocument {
    schema_version: u32,
    armature: ArmatureMetadata,
    body_track_names: Vec<String>,
    joint_marker_track_names: Vec<String>,
    bones: Vec<RigBone>,
}

#[derive(Debug, Deserialize)]
struct ArmatureMetadata {
    matrix_world: [[f32; 4]; 4],
}

#[derive(Clone, Copy, Debug)]
pub struct HumanBonePose {
    pub rotation_engine: Quat,
    pub center_engine: Vec3,
    pub collision: BoneCollisionGeometryValue,
}

#[derive(Clone, Copy, Debug)]
pub struct BoneCollisionGeometryValue {
    pub radius_m: f32,
    pub capsule_segment_length_m: f32,
}

impl HumanRig {
    pub fn load(data_dir: &Path) -> Result<Self> {
        let path = data_dir.join("human").join("rig.json");
        let source = fs::read_to_string(&path)
            .with_context(|| format!("reading generated human rig metadata {}", path.display()))?;
        Self::from_json(&source)
            .with_context(|| format!("validating generated human rig metadata {}", path.display()))
    }

    fn from_json(source: &str) -> Result<Self> {
        let document: RigDocument = serde_json::from_str(source)?;
        ensure!(
            document.schema_version == 1,
            "unsupported human rig schema version {}",
            document.schema_version
        );
        ensure!(!document.bones.is_empty(), "human rig contains no bones");

        let armature_world = matrix_from_rows(&document.armature.matrix_world);
        ensure!(
            matrix_is_finite(armature_world),
            "human armature transform contains non-finite values"
        );

        let body_names: BTreeSet<_> = document.body_track_names.into_iter().collect();
        let marker_names: BTreeSet<_> = document.joint_marker_track_names.into_iter().collect();
        ensure!(!body_names.is_empty(), "human rig contains no body tracks");
        ensure!(
            body_names.contains("Root"),
            "human rig has no Root body track"
        );

        let mut bones = BTreeMap::new();
        for bone in document.bones {
            ensure!(
                !bone.name.trim().is_empty(),
                "human rig contains an empty bone name"
            );
            ensure!(
                bone.length_m.is_finite() && bone.length_m > 0.0,
                "human rig bone '{}' has invalid length {}",
                bone.name,
                bone.length_m
            );
            let rest = matrix_from_rows(&bone.matrix_parent_local);
            ensure!(
                matrix_is_finite(rest),
                "human rig bone '{}' has a non-finite rest transform",
                bone.name
            );
            if let Some(reference) = bone.action_reference_parent_local {
                let reference = matrix_from_flat_row_major(&reference);
                ensure!(
                    matrix_is_finite(reference),
                    "human rig bone '{}' has a non-finite idle reference transform",
                    bone.name
                );
                ensure!(
                    reference.determinant().abs() > 1.0e-8,
                    "human rig bone '{}' has a singular idle reference transform",
                    bone.name
                );
            }
            if body_names.contains(&bone.name) {
                ensure!(
                    bone.deform,
                    "body track '{}' is not a deformation bone",
                    bone.name
                );
                let collision = bone.collision_geometry.as_ref().with_context(|| {
                    format!(
                        "body track '{}' has no mesh-derived collision geometry",
                        bone.name
                    )
                })?;
                ensure!(
                    collision.shape == "capsule",
                    "body track '{}' has unsupported collision shape '{}'",
                    bone.name,
                    collision.shape
                );
                ensure!(
                    collision.radius_m.is_finite()
                        && collision.radius_m > 0.0
                        && collision.axial_min_m.is_finite()
                        && collision.axial_max_m.is_finite()
                        && collision.axial_max_m > collision.axial_min_m
                        && collision.axial_length_m.is_finite()
                        && collision.axial_length_m > 0.0
                        && (collision.axial_length_m
                            - (collision.axial_max_m - collision.axial_min_m))
                            .abs()
                            <= 1.0e-4
                        && collision.capsule_segment_length_m.is_finite()
                        && collision.capsule_segment_length_m >= 0.0
                        && (collision.capsule_segment_length_m
                            - (collision.axial_length_m - 2.0 * collision.radius_m).max(0.0))
                            .abs()
                            <= 1.0e-4
                        && collision.center_offset_bone_m.iter().all(|x| x.is_finite())
                        && collision.weight_threshold.is_finite()
                        && (0.0..=1.0).contains(&collision.weight_threshold)
                        && collision.weighted_vertex_count >= 4,
                    "body track '{}' has invalid mesh-derived collision dimensions",
                    bone.name
                );
            }
            ensure!(
                bones.insert(bone.name.clone(), bone).is_none(),
                "human rig contains a duplicate bone name"
            );
        }

        for body_name in &body_names {
            ensure!(
                bones.contains_key(body_name),
                "body track '{body_name}' has no deformation bone"
            );
            ensure!(
                bones[body_name].action_reference_parent_local.is_some(),
                "body track '{body_name}' has no idle action reference transform"
            );
        }
        for marker_name in &marker_names {
            ensure!(
                bones.contains_key(marker_name),
                "joint marker '{marker_name}' has no rig bone"
            );
            ensure!(
                bones[marker_name].action_reference_parent_local.is_some(),
                "joint marker '{marker_name}' has no idle action reference transform"
            );
        }

        // The converter stores body-to-marker assignments separately from marker self-tags.
        // Validate exact coverage and the helper-bone hierarchy before spawning physics joints.
        let mut assigned_markers = BTreeSet::new();
        for body_name in &body_names {
            if let Some(marker_name) = &bones[body_name].joint_anchor_track {
                ensure!(
                    marker_names.contains(marker_name),
                    "body track '{body_name}' references undeclared joint marker '{marker_name}'"
                );
                let marker_bone = &bones[marker_name];
                ensure!(
                    marker_bone.joint_marker_track.as_deref() == Some(marker_name.as_str()),
                    "body track '{body_name}' references '{marker_name}', which is not a tagged joint-marker bone"
                );
                let body_parent = bones[body_name].parent.as_deref();
                ensure!(
                    body_parent == Some(marker_name.as_str())
                        || body_parent == marker_bone.parent.as_deref(),
                    "joint marker '{marker_name}' is not a helper of body track '{body_name}'"
                );
                ensure!(
                    assigned_markers.insert(marker_name.clone()),
                    "joint marker '{marker_name}' is assigned to more than one body track"
                );
            }
        }
        ensure!(
            assigned_markers == marker_names,
            "human rig joint markers do not have a one-to-one body-track assignment"
        );
        for bone in bones.values() {
            if let Some(parent) = &bone.parent {
                ensure!(
                    bones.contains_key(parent),
                    "bone '{}' references missing parent '{}'",
                    bone.name,
                    parent
                );
            }
        }

        // Detect malformed parent cycles once, before the simulation starts.
        for name in bones.keys() {
            let mut visited = BTreeSet::new();
            let mut current = Some(name.as_str());
            while let Some(candidate) = current {
                ensure!(
                    visited.insert(candidate.to_owned()),
                    "human rig hierarchy contains a parent cycle at '{candidate}'"
                );
                current = bones.get(candidate).and_then(|bone| bone.parent.as_deref());
            }
        }

        Ok(Self {
            armature_world,
            bones,
            body_names,
            marker_names,
        })
    }

    pub fn body_names(&self) -> impl Iterator<Item = &str> {
        self.body_names.iter().map(String::as_str)
    }

    pub fn marker_names(&self) -> impl Iterator<Item = &str> {
        self.marker_names.iter().map(String::as_str)
    }

    /// Returns the rig-metadata joint marker assigned to a physical body segment.
    pub fn joint_marker_for_body(&self, body_name: &str) -> Option<&str> {
        self.bones.get(body_name)?.joint_anchor_track.as_deref()
    }

    pub fn bone_names(&self) -> impl Iterator<Item = &str> {
        self.bones.keys().map(String::as_str)
    }

    pub fn bone(&self, name: &str) -> Option<&RigBone> {
        self.bones.get(name)
    }

    /// Samples calibrated transforms relative to each bone's actual parent.
    /// Returned matrices remain in the source rig's Z-up coordinate basis.
    pub fn sample_calibrated_local_pose(
        &self,
        action: &HumanAction,
        elapsed_seconds: f32,
        reverse: bool,
    ) -> Result<BTreeMap<String, Mat4>> {
        let mut local_poses = BTreeMap::new();
        for (name, bone) in &self.bones {
            let rest_local = matrix_from_rows(&bone.matrix_parent_local);
            let local = match (
                action.track(name),
                bone.action_reference_parent_local.as_ref(),
            ) {
                (Some(track), Some(reference)) => {
                    let idle_reference = matrix_from_flat_row_major(reference);
                    let sampled = track.sample_matrix(elapsed_seconds, action.looping, reverse);
                    rest_local * idle_reference.inverse() * sampled
                }
                (Some(_), None) => {
                    anyhow::bail!("human rig bone '{name}' has no idle calibration transform")
                }
                (None, _) => rest_local,
            };
            ensure!(
                matrix_is_finite(local),
                "calibrated local transform for bone '{name}' is not finite"
            );
            local_poses.insert(name.clone(), local);
        }
        Ok(local_poses)
    }

    /// Calibrates each action track's idle matrix to its FBX rest-local matrix.
    ///
    /// The source files share bone names but differ slightly in local rest conventions.
    /// This correction makes the idle action reproduce the actual FBX rest pose while
    /// preserving subsequent per-track transforms as physical pose targets.
    pub fn sample_source_world_pose(
        &self,
        action: &HumanAction,
        elapsed_seconds: f32,
        reverse: bool,
    ) -> Result<BTreeMap<String, Mat4>> {
        let local_poses = self.sample_calibrated_local_pose(action, elapsed_seconds, reverse)?;
        let mut world = BTreeMap::new();
        let mut pending: BTreeSet<String> = self.bones.keys().cloned().collect();

        while !pending.is_empty() {
            let mut progressed = false;
            let names: Vec<_> = pending.iter().cloned().collect();
            for name in names {
                let bone = &self.bones[&name];
                let parent_world = match &bone.parent {
                    Some(parent) => {
                        let Some(parent_matrix) = world.get(parent) else {
                            continue;
                        };
                        *parent_matrix
                    }
                    None => self.armature_world,
                };
                world.insert(name.clone(), parent_world * local_poses[&name]);
                pending.remove(&name);
                progressed = true;
            }
            ensure!(
                progressed,
                "human rig hierarchy could not be resolved (missing ancestor or cycle)"
            );
        }

        Ok(world)
    }

    /// Converts source Z-up/+Y-front body poses to engine Y-up/-Z-forward coordinates.
    pub fn sample_engine_body_pose(
        &self,
        action: &HumanAction,
        elapsed_seconds: f32,
        reverse: bool,
    ) -> Result<BTreeMap<String, HumanBonePose>> {
        let source_pose = self.sample_source_world_pose(action, elapsed_seconds, reverse)?;
        let source_to_engine = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let mut result = BTreeMap::new();

        for name in &self.body_names {
            let bone = &self.bones[name];
            let geometry = bone
                .collision_geometry
                .as_ref()
                .context("body collision geometry was validated during rig load")?;
            let source_matrix = source_pose[name];
            let source_head = source_matrix.transform_point3(Vec3::ZERO);
            let source_axis = source_matrix.transform_vector3(Vec3::Y).normalize_or_zero();
            ensure!(
                source_axis.length_squared() > 0.99,
                "human body bone '{name}' has a degenerate longitudinal axis"
            );

            let (_, source_rotation, _) = source_matrix.to_scale_rotation_translation();
            let rotation_engine = (source_to_engine * source_rotation).normalize();
            let head_engine = source_to_engine * source_head;
            let center_offset = Vec3::from_array(geometry.center_offset_bone_m);
            let center_engine = head_engine + rotation_engine * center_offset;

            result.insert(
                name.clone(),
                HumanBonePose {
                    rotation_engine,
                    center_engine,
                    collision: BoneCollisionGeometryValue {
                        radius_m: geometry.radius_m,
                        capsule_segment_length_m: geometry.capsule_segment_length_m,
                    },
                },
            );
        }

        Ok(result)
    }

    /// Converts a calibrated source-space joint-anchor bone origin into engine coordinates.
    pub fn joint_anchor_position_engine(
        &self,
        source_world_pose: &BTreeMap<String, Mat4>,
        bone_name: &str,
    ) -> Option<Vec3> {
        let matrix = source_world_pose.get(bone_name)?;
        let source_position = matrix.transform_point3(Vec3::ZERO);
        Some(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2) * source_position)
    }
}

fn matrix_from_rows(rows: &[[f32; 4]; 4]) -> Mat4 {
    let flat = [
        rows[0][0], rows[0][1], rows[0][2], rows[0][3], rows[1][0], rows[1][1], rows[1][2],
        rows[1][3], rows[2][0], rows[2][1], rows[2][2], rows[2][3], rows[3][0], rows[3][1],
        rows[3][2], rows[3][3],
    ];
    Mat4::from_cols_array(&flat).transpose()
}

fn matrix_from_flat_row_major(values: &[f32; 16]) -> Mat4 {
    Mat4::from_cols_array(values).transpose()
}

fn matrix_is_finite(matrix: Mat4) -> bool {
    matrix.to_cols_array().iter().all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
    use super::HumanRig;
    use crate::human::HumanMotionLibrary;
    use std::{collections::BTreeSet, path::Path};

    #[test]
    fn loads_all_deformation_bodies_and_joint_markers() {
        let rig = HumanRig::load(Path::new("data")).unwrap();
        assert_eq!(rig.body_names().count(), 12);
        assert_eq!(rig.marker_names().count(), 6);
        let assigned_markers: BTreeSet<_> = rig
            .body_names()
            .filter_map(|name| rig.joint_marker_for_body(name))
            .collect();
        assert_eq!(assigned_markers.len(), 6);
        for name in rig.body_names() {
            let bone = rig.bone(name).unwrap();
            assert!(bone.collision_geometry.is_some(), "{name}");
        }
    }

    #[test]
    fn calibrated_idle_reproduces_imported_rest_pose() {
        let rig = HumanRig::load(Path::new("data")).unwrap();
        let actions = HumanMotionLibrary::load(Path::new("data")).unwrap();
        let idle = actions.action("idle").unwrap();
        let world = rig.sample_source_world_pose(idle, 0.0, false).unwrap();
        for (name, bone) in &rig.bones {
            let actual = world[name];
            let mut expected = super::matrix_from_rows(&bone.matrix_parent_local);
            let mut parent = bone.parent.as_deref();
            while let Some(parent_name) = parent {
                expected =
                    super::matrix_from_rows(&rig.bone(parent_name).unwrap().matrix_parent_local)
                        * expected;
                parent = rig.bone(parent_name).unwrap().parent.as_deref();
            }
            if bone.parent.is_none() {
                expected = rig.armature_world * expected;
            }
            let difference = actual
                .to_cols_array()
                .iter()
                .zip(expected.to_cols_array())
                .map(|(a, b)| (a - b).abs())
                .fold(0.0_f32, f32::max);
            assert!(
                difference < 1.0e-4,
                "idle calibration mismatch for {name}: {difference}"
            );
        }
    }

    #[test]
    fn coordinate_conversion_maps_source_up_and_front_correctly() {
        let conversion = bevy::prelude::Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let source_up = conversion * bevy::prelude::Vec3::Z;
        let source_front = conversion * bevy::prelude::Vec3::Y;
        assert!((source_up - bevy::prelude::Vec3::Y).length() < 1.0e-5);
        assert!((source_front + bevy::prelude::Vec3::Z).length() < 1.0e-5);
    }
}

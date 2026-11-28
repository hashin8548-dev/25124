use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::Path,
};

use anyhow::{Result, ensure};
use avian3d::prelude::{
    AngularVelocity, Collider, ColliderDensity, CollisionEventsEnabled, CollisionLayers, Friction,
    JointCollisionDisabled, JointDamping, LinearVelocity, Restitution, RevoluteJoint, RigidBody,
    SphericalJoint,
};
use bevy::{gltf::prelude::GltfAssetLabel, math::Isometry3d, prelude::*};
use serde::Deserialize;

use crate::world::WorldOrigin;

use super::{HumanActionState, HumanMotionLibrary, HumanRig};

/// Minimum two-hand friction capacity relative to body weight for a climb hold.
const MIN_CLIMB_SUPPORT_FACTOR: f32 = 1.2;

#[derive(Clone, Debug, Resource)]
pub struct HumanPhysicsSettings {
    pub total_mass_kg: f32,
    pub mass_distribution_reference: String,
    pub body_mass_fractions: BTreeMap<String, f32>,
    pub static_friction: f32,
    pub dynamic_friction: f32,
    pub restitution: f32,
    pub joint_damping_linear: f32,
    pub joint_damping_angular: f32,
    pub joint_point_compliance: f32,
    pub joint_angular_compliance: f32,
    pub pose_stiffness_nm_per_rad: f32,
    pub pose_damping_nm_s_per_rad: f32,
    pub max_pose_torque_nm: f32,
    pub pose_position_stiffness_n_per_m: f32,
    pub pose_position_damping_n_s_per_m: f32,
    pub max_pose_force_n: f32,
    pub climb_contact_push_force_n: f32,
    pub self_collision_enabled: bool,
    pub self_collision_overlap_margin_m: f32,
    pub temporary_assists: HumanTemporaryAssists,
    pub controls: HumanControlSettings,
    pub camera: HumanCameraSettings,
    pub limits: HumanJointLimits,
}

#[derive(Clone, Debug)]
pub struct HumanCameraSettings {
    pub trailing_distance_m: f32,
    pub shoulder_offset_m: [f32; 3],
    pub look_at_offset_m: [f32; 3],
    pub vertical_fov_degrees: f32,
    pub spring_stiffness: f32,
    pub spring_damping: f32,
    pub crosshair_diameter_px: f32,
}

#[derive(Clone, Debug)]
pub struct HumanControlSettings {
    pub walk: String,
    pub run: String,
    pub crouch: String,
    pub crouched_idle: String,
    pub jump: String,
    pub climb: String,
    pub fall: String,
    pub idle: String,
    pub walk_backwards_reverse: bool,
    pub crouch_backwards_reverse: bool,
    pub bindings: HumanInputBindings,
}

#[derive(Clone, Debug)]
pub struct HumanInputBindings {
    pub forward: KeyCode,
    pub backward: KeyCode,
    pub left: KeyCode,
    pub right: KeyCode,
    pub run_modifiers: Vec<KeyCode>,
    pub crouch: KeyCode,
    pub jump: KeyCode,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HumanTemporaryAssists {
    pub jump_launch: JumpLaunchAssist,
    pub rotational_assistance: RotationalAssist,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JumpLaunchAssist {
    pub kind: String,
    pub enabled: bool,
    pub launch_speed_m_s: f32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RotationalAssist {
    pub kind: String,
    pub enabled: bool,
    pub strength: f32,
    pub max_torque_nm: f32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HumanJointLimits {
    pub hip_swing_rad: f32,
    pub hip_twist_rad: f32,
    pub shoulder_swing_rad: f32,
    pub shoulder_twist_rad: f32,
    pub spine_swing_rad: f32,
    pub spine_twist_rad: f32,
    pub neck_swing_rad: f32,
    pub neck_twist_rad: f32,
    pub knee_min_rad: f32,
    pub knee_max_rad: f32,
    pub elbow_min_rad: f32,
    pub elbow_max_rad: f32,
}

#[derive(Debug, Deserialize)]
struct HumanManifest {
    schema_version: u32,
    physics: HumanPhysicsSettingsFile,
    temporary_assists: HumanTemporaryAssists,
    controls: serde_json::Value,
    camera: serde_json::Value,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct HumanPhysicsSettingsFile {
    total_mass_kg: f32,
    mass_distribution_reference: String,
    body_mass_fractions: BTreeMap<String, f32>,
    static_friction: f32,
    dynamic_friction: f32,
    restitution: f32,
    joint_damping_linear: f32,
    joint_damping_angular: f32,
    joint_point_compliance: f32,
    joint_angular_compliance: f32,
    pose_stiffness_nm_per_rad: f32,
    pose_damping_nm_s_per_rad: f32,
    max_pose_torque_nm: f32,
    pose_position_stiffness_n_per_m: f32,
    pose_position_damping_n_s_per_m: f32,
    max_pose_force_n: f32,
    climb_contact_push_force_n: f32,
    self_collision_enabled: bool,
    self_collision_overlap_margin_m: f32,
    limits: HumanJointLimits,
}

#[derive(Component, Clone, Debug)]
pub struct HumanBodyPart {
    pub bone_name: String,
}

#[derive(Component, Clone, Copy, Debug)]
pub struct HumanPlayerRoot;

/// Marks the Bevy world instance containing the human deformation skeleton.
#[derive(Component, Clone, Copy, Debug)]
pub struct HumanSkinnedSceneRoot;

/// Marker for standalone Avian joint entities so they are removed with the player assembly.
#[derive(Component, Clone, Copy, Debug)]
pub(super) struct HumanJointConstraint;

#[derive(Clone, Debug, Resource)]
pub struct HumanPlayer {
    pub root: Entity,
    pub bodies: BTreeMap<String, Entity>,
}

fn input_binding_name<'a>(controls: &'a serde_json::Value, name: &str) -> Result<&'a str> {
    let value = controls
        .get("bindings")
        .and_then(|bindings| bindings.get(name))
        .and_then(serde_json::Value::as_str);
    anyhow::Context::with_context(value, || {
        format!("human controls are missing key binding '{name}'")
    })
}

fn key_codes_for_binding(name: &str) -> Result<Vec<KeyCode>> {
    let single = match name {
        "A" => KeyCode::KeyA,
        "B" => KeyCode::KeyB,
        "C" => KeyCode::KeyC,
        "D" => KeyCode::KeyD,
        "E" => KeyCode::KeyE,
        "F" => KeyCode::KeyF,
        "G" => KeyCode::KeyG,
        "H" => KeyCode::KeyH,
        "I" => KeyCode::KeyI,
        "J" => KeyCode::KeyJ,
        "K" => KeyCode::KeyK,
        "L" => KeyCode::KeyL,
        "M" => KeyCode::KeyM,
        "N" => KeyCode::KeyN,
        "O" => KeyCode::KeyO,
        "P" => KeyCode::KeyP,
        "Q" => KeyCode::KeyQ,
        "R" => KeyCode::KeyR,
        "S" => KeyCode::KeyS,
        "T" => KeyCode::KeyT,
        "U" => KeyCode::KeyU,
        "V" => KeyCode::KeyV,
        "W" => KeyCode::KeyW,
        "X" => KeyCode::KeyX,
        "Y" => KeyCode::KeyY,
        "Z" => KeyCode::KeyZ,
        "SPACE" => KeyCode::Space,
        "SHIFT_LEFT" => KeyCode::ShiftLeft,
        "SHIFT_RIGHT" => KeyCode::ShiftRight,
        "ARROW_UP" => KeyCode::ArrowUp,
        "ARROW_DOWN" => KeyCode::ArrowDown,
        "ARROW_LEFT" => KeyCode::ArrowLeft,
        "ARROW_RIGHT" => KeyCode::ArrowRight,
        _ => {
            if name == "SHIFT" {
                return Ok(vec![KeyCode::ShiftLeft, KeyCode::ShiftRight]);
            }
            anyhow::bail!("unsupported human key binding '{name}'");
        }
    };
    Ok(vec![single])
}

fn required_input_binding(controls: &serde_json::Value, name: &str) -> Result<Vec<KeyCode>> {
    key_codes_for_binding(input_binding_name(controls, name)?)
}

fn single_input_binding(controls: &serde_json::Value, name: &str) -> Result<KeyCode> {
    let mut keys = required_input_binding(controls, name)?;
    ensure!(
        keys.len() == 1,
        "human key binding '{name}' must resolve to exactly one key"
    );
    Ok(keys.remove(0))
}

fn validate_action_key_set(
    controls: &serde_json::Value,
    action: &str,
    bindings: &[&str],
) -> Result<()> {
    let expected: BTreeSet<_> = bindings
        .iter()
        .map(|name| input_binding_name(controls, name).map(str::to_owned))
        .collect::<Result<_>>()?;
    let values = controls
        .get(action)
        .and_then(|entry| entry.get("keys"))
        .and_then(serde_json::Value::as_array);
    let values = anyhow::Context::with_context(values, || {
        format!("human control '{action}' must declare its key list")
    })?;
    let actual: BTreeSet<_> = values
        .iter()
        .map(|value| {
            value.as_str().map(str::to_owned).ok_or_else(|| {
                anyhow::anyhow!("human control '{action}' contains a non-string key")
            })
        })
        .collect::<Result<_>>()?;
    ensure!(
        actual.len() == values.len() && actual == expected,
        "human control '{action}' keys do not match semantic bindings"
    );
    Ok(())
}

fn parse_human_input_bindings(controls: &serde_json::Value) -> Result<HumanInputBindings> {
    validate_action_key_set(controls, "walk", &["forward", "backward", "left", "right"])?;
    validate_action_key_set(controls, "run", &["run", "forward", "left", "right"])?;
    validate_action_key_set(
        controls,
        "crouch",
        &["crouch", "forward", "backward", "left", "right"],
    )?;
    validate_action_key_set(controls, "crouched_idle", &["crouch"])?;
    validate_action_key_set(controls, "jump", &["jump"])?;

    Ok(HumanInputBindings {
        forward: single_input_binding(controls, "forward")?,
        backward: single_input_binding(controls, "backward")?,
        left: single_input_binding(controls, "left")?,
        right: single_input_binding(controls, "right")?,
        run_modifiers: required_input_binding(controls, "run")?,
        crouch: single_input_binding(controls, "crouch")?,
        jump: single_input_binding(controls, "jump")?,
    })
}

impl HumanPhysicsSettings {
    pub fn load(data_dir: &Path) -> Result<Self> {
        let path = data_dir.join("human").join("human.json");
        let text = anyhow::Context::with_context(fs::read_to_string(&path), || {
            format!("reading human physics settings {}", path.display())
        })?;
        let manifest: HumanManifest =
            anyhow::Context::with_context(serde_json::from_str(&text), || {
                format!("parsing human physics settings {}", path.display())
            })?;
        ensure!(
            manifest.schema_version == 1,
            "unsupported human manifest schema version"
        );
        let source = manifest.physics;
        ensure!(
            source.total_mass_kg.is_finite()
                && source.total_mass_kg > 0.0
                && source.total_mass_kg <= 500.0,
            "human total body mass must be in (0, 500] kg"
        );
        ensure!(
            !source.mass_distribution_reference.trim().is_empty(),
            "human mass distribution must identify its anthropometric reference"
        );
        ensure!(
            !source.body_mass_fractions.is_empty()
                && source.body_mass_fractions.values().all(|fraction| {
                    fraction.is_finite() && *fraction > 0.0 && *fraction <= 1.0
                }),
            "human segment mass fractions must be positive and finite"
        );
        let mass_fraction_sum: f32 = source.body_mass_fractions.values().sum();
        ensure!(
            (mass_fraction_sum - 1.0).abs() <= 1.0e-4,
            "human segment mass fractions must sum to 1.0 (got {mass_fraction_sum})"
        );
        ensure!(
            source.static_friction.is_finite()
                && source.dynamic_friction.is_finite()
                && source.static_friction >= source.dynamic_friction
                && source.dynamic_friction >= 0.0,
            "human friction coefficients are invalid"
        );
        ensure!(
            source.restitution.is_finite() && (0.0..=1.0).contains(&source.restitution),
            "human restitution must be in [0, 1]"
        );
        ensure!(
            source.joint_damping_linear.is_finite()
                && source.joint_damping_linear >= 0.0
                && source.joint_damping_angular.is_finite()
                && source.joint_damping_angular >= 0.0,
            "human joint damping values must be non-negative and finite"
        );
        ensure!(
            source.joint_point_compliance.is_finite()
                && source.joint_point_compliance >= 0.0
                && source.joint_angular_compliance.is_finite()
                && source.joint_angular_compliance >= 0.0,
            "human joint compliance values must be non-negative and finite"
        );
        ensure!(
            source.pose_stiffness_nm_per_rad.is_finite()
                && source.pose_stiffness_nm_per_rad > 0.0
                && source.pose_damping_nm_s_per_rad.is_finite()
                && source.pose_damping_nm_s_per_rad >= 0.0
                && source.max_pose_torque_nm.is_finite()
                && source.max_pose_torque_nm > 0.0,
            "human pose-actuation settings are invalid"
        );
        ensure!(
            manifest.temporary_assists.jump_launch.kind == "impulse"
                && manifest.temporary_assists.rotational_assistance.kind == "bounded_torque_pd",
            "human temporary-assist kind is unsupported"
        );
        ensure!(
            manifest
                .temporary_assists
                .jump_launch
                .launch_speed_m_s
                .is_finite()
                && manifest.temporary_assists.jump_launch.launch_speed_m_s >= 0.0
                && manifest
                    .temporary_assists
                    .rotational_assistance
                    .strength
                    .is_finite()
                && manifest.temporary_assists.rotational_assistance.strength >= 0.0
                && manifest
                    .temporary_assists
                    .rotational_assistance
                    .max_torque_nm
                    .is_finite()
                && manifest
                    .temporary_assists
                    .rotational_assistance
                    .max_torque_nm
                    >= 0.0,
            "human temporary-assist settings are invalid"
        );
        ensure!(
            source.pose_position_stiffness_n_per_m.is_finite()
                && source.pose_position_stiffness_n_per_m > 0.0
                && source.pose_position_damping_n_s_per_m.is_finite()
                && source.pose_position_damping_n_s_per_m >= 0.0
                && source.max_pose_force_n.is_finite()
                && source.max_pose_force_n > 0.0,
            "human position-actuation settings are invalid"
        );
        ensure!(
            source.climb_contact_push_force_n.is_finite()
                && source.climb_contact_push_force_n >= 0.0,
            "human climb contact force must be non-negative and finite"
        );
        let two_hand_climb_friction_n = source.dynamic_friction * source.climb_contact_push_force_n;
        let body_weight_n = source.total_mass_kg * 9.81;
        ensure!(
            two_hand_climb_friction_n >= body_weight_n * MIN_CLIMB_SUPPORT_FACTOR,
            "two-hand climb friction capacity ({two_hand_climb_friction_n:.1} N) must exceed body weight ({body_weight_n:.1} N) by the configured support factor"
        );
        ensure!(
            source.self_collision_overlap_margin_m.is_finite()
                && (0.0..=0.1).contains(&source.self_collision_overlap_margin_m),
            "human self-collision overlap margin must be finite and in [0, 0.1] m"
        );
        let controls = &manifest.controls;
        let action_name = |state: &str| -> Result<String> {
            let action = controls
                .get(state)
                .and_then(|entry| entry.get("action"))
                .and_then(serde_json::Value::as_str)
                .map(str::to_owned);
            anyhow::Context::with_context(action, || {
                format!("human control state '{state}' has no action name")
            })
        };
        let walk_backwards_reverse = controls
            .get("walk")
            .and_then(|entry| entry.get("backward"))
            .and_then(|entry| entry.get("time_direction"))
            .and_then(serde_json::Value::as_str)
            == Some("reverse");
        let crouch_backwards_reverse = controls
            .get("crouch")
            .and_then(|entry| entry.get("backward"))
            .and_then(|entry| entry.get("time_direction"))
            .and_then(serde_json::Value::as_str)
            == Some("reverse");
        ensure!(
            walk_backwards_reverse && crouch_backwards_reverse,
            "human backwards walk and crouch must use reverse-time source motions"
        );
        ensure!(
            controls
                .get("run")
                .and_then(|entry| entry.get("backward"))
                .and_then(serde_json::Value::as_bool)
                == Some(false),
            "human run must explicitly disable backwards running"
        );
        ensure!(
            controls
                .get("jump")
                .and_then(|entry| entry.get("when_wall_ahead"))
                .and_then(serde_json::Value::as_str)
                == Some("climb"),
            "human jump must select climb when a wall is ahead"
        );
        let control_settings = HumanControlSettings {
            walk: action_name("walk")?,
            run: action_name("run")?,
            crouch: action_name("crouch")?,
            crouched_idle: action_name("crouched_idle")?,
            jump: action_name("jump")?,
            climb: action_name("climb")?,
            fall: action_name("fall")?,
            idle: action_name("idle")?,
            walk_backwards_reverse,
            crouch_backwards_reverse,
            bindings: parse_human_input_bindings(controls)?,
        };
        let camera = &manifest.camera;
        let camera_number = |name: &str| -> Result<f32> {
            let value = camera
                .get(name)
                .and_then(serde_json::Value::as_f64)
                .map(|value| value as f32)
                .filter(|value| value.is_finite());
            anyhow::Context::with_context(value, || {
                format!("human camera setting '{name}' must be finite")
            })
        };
        let camera_vec3 = |name: &str| -> Result<[f32; 3]> {
            let values = camera.get(name).and_then(serde_json::Value::as_array);
            let values = anyhow::Context::with_context(values, || {
                format!("human camera setting '{name}' must be a 3D vector")
            })?;
            ensure!(
                values.len() == 3,
                "human camera setting '{name}' must have three components"
            );
            let mut result = [0.0; 3];
            for (index, value) in values.iter().enumerate() {
                let number = value
                    .as_f64()
                    .map(|number| number as f32)
                    .filter(|number| number.is_finite());
                result[index] = anyhow::Context::with_context(number, || {
                    format!("human camera setting '{name}[{index}]' must be finite")
                })?;
            }
            Ok(result)
        };
        ensure!(
            camera.get("mode").and_then(serde_json::Value::as_str)
                == Some("third_person_over_shoulder"),
            "unsupported human camera mode"
        );
        let crosshair = anyhow::Context::context(
            camera.get("crosshair"),
            "human camera crosshair settings are missing",
        )?;
        ensure!(
            crosshair.get("kind").and_then(serde_json::Value::as_str) == Some("single_dot"),
            "human camera crosshair must be a single dot"
        );
        let camera_settings = HumanCameraSettings {
            trailing_distance_m: camera_number("trailing_distance_m")?,
            shoulder_offset_m: camera_vec3("shoulder_offset_m")?,
            look_at_offset_m: camera_vec3("look_at_offset_m")?,
            vertical_fov_degrees: camera_number("vertical_fov_degrees")?,
            spring_stiffness: camera_number("spring_stiffness")?,
            spring_damping: camera_number("spring_damping")?,
            crosshair_diameter_px: anyhow::Context::context(
                crosshair
                    .get("diameter_px")
                    .and_then(serde_json::Value::as_f64)
                    .map(|value| value as f32)
                    .filter(|value| value.is_finite()),
                "human crosshair diameter must be finite",
            )?,
        };
        ensure!(
            camera_settings.trailing_distance_m > 0.0
                && (30.0..=120.0).contains(&camera_settings.vertical_fov_degrees)
                && camera_settings.spring_stiffness > 0.0
                && camera_settings.spring_damping >= 0.0
                && camera_settings.crosshair_diameter_px >= 1.0
                && camera_settings.crosshair_diameter_px <= 12.0,
            "human camera settings are outside supported ranges"
        );
        let limits = &source.limits;
        for (name, value) in [
            ("hip swing", limits.hip_swing_rad),
            ("hip twist", limits.hip_twist_rad),
            ("shoulder swing", limits.shoulder_swing_rad),
            ("shoulder twist", limits.shoulder_twist_rad),
            ("spine swing", limits.spine_swing_rad),
            ("spine twist", limits.spine_twist_rad),
            ("neck swing", limits.neck_swing_rad),
            ("neck twist", limits.neck_twist_rad),
        ] {
            ensure!(
                value.is_finite() && (0.0..=std::f32::consts::PI).contains(&value),
                "human {name} limit is invalid"
            );
        }
        ensure!(
            limits.knee_min_rad.is_finite()
                && limits.knee_max_rad.is_finite()
                && limits.knee_min_rad < limits.knee_max_rad
                && limits.elbow_min_rad.is_finite()
                && limits.elbow_max_rad.is_finite()
                && limits.elbow_min_rad < limits.elbow_max_rad,
            "human hinge limits are invalid"
        );

        Ok(Self {
            total_mass_kg: source.total_mass_kg,
            mass_distribution_reference: source.mass_distribution_reference,
            body_mass_fractions: source.body_mass_fractions,
            static_friction: source.static_friction,
            dynamic_friction: source.dynamic_friction,
            restitution: source.restitution,
            joint_damping_linear: source.joint_damping_linear,
            joint_damping_angular: source.joint_damping_angular,
            joint_point_compliance: source.joint_point_compliance,
            joint_angular_compliance: source.joint_angular_compliance,
            pose_stiffness_nm_per_rad: source.pose_stiffness_nm_per_rad,
            pose_damping_nm_s_per_rad: source.pose_damping_nm_s_per_rad,
            max_pose_torque_nm: source.max_pose_torque_nm,
            pose_position_stiffness_n_per_m: source.pose_position_stiffness_n_per_m,
            pose_position_damping_n_s_per_m: source.pose_position_damping_n_s_per_m,
            max_pose_force_n: source.max_pose_force_n,
            climb_contact_push_force_n: source.climb_contact_push_force_n,
            self_collision_enabled: source.self_collision_enabled,
            self_collision_overlap_margin_m: source.self_collision_overlap_margin_m,
            temporary_assists: manifest.temporary_assists,
            controls: control_settings,
            camera: camera_settings,
            limits: source.limits,
        })
    }

    pub fn body_mass_kg(&self, body_name: &str) -> Option<f32> {
        self.body_mass_fractions
            .get(body_name)
            .map(|fraction| self.total_mass_kg * fraction)
    }
}

#[derive(Clone, Copy, Debug)]
struct CapsuleCenterline {
    start: Vec3,
    end: Vec3,
    radius: f32,
}

fn capsule_centerline(pose: &super::rig::HumanBonePose) -> CapsuleCenterline {
    let axis = (pose.rotation_engine * Vec3::Y).normalize();
    let half_segment = pose.collision.capsule_segment_length_m * 0.5;
    CapsuleCenterline {
        start: pose.center_engine - axis * half_segment,
        end: pose.center_engine + axis * half_segment,
        radius: pose.collision.radius_m,
    }
}

fn segment_segment_distance(
    first_start: Vec3,
    first_end: Vec3,
    second_start: Vec3,
    second_end: Vec3,
) -> f32 {
    let first_direction = first_end - first_start;
    let second_direction = second_end - second_start;
    let offset = first_start - second_start;
    let first_length_squared = first_direction.length_squared();
    let second_length_squared = second_direction.length_squared();
    let second_projection = second_direction.dot(offset);
    let mut first_parameter = 0.0;
    let mut second_parameter = 0.0;

    if first_length_squared <= 1.0e-12 && second_length_squared <= 1.0e-12 {
        return first_start.distance(second_start);
    }
    if first_length_squared <= 1.0e-12 {
        second_parameter = (second_projection / second_length_squared).clamp(0.0, 1.0);
    } else {
        let first_projection = first_direction.dot(offset);
        if second_length_squared <= 1.0e-12 {
            first_parameter = (-first_projection / first_length_squared).clamp(0.0, 1.0);
        } else {
            let cross_projection = first_direction.dot(second_direction);
            let denominator =
                first_length_squared * second_length_squared - cross_projection * cross_projection;
            if denominator.abs() > 1.0e-12 {
                first_parameter = ((cross_projection * second_projection
                    - first_projection * second_length_squared)
                    / denominator)
                    .clamp(0.0, 1.0);
            }
            second_parameter =
                (cross_projection * first_parameter + second_projection) / second_length_squared;
            if second_parameter < 0.0 {
                second_parameter = 0.0;
                first_parameter = (-first_projection / first_length_squared).clamp(0.0, 1.0);
            } else if second_parameter > 1.0 {
                second_parameter = 1.0;
                first_parameter =
                    ((cross_projection - first_projection) / first_length_squared).clamp(0.0, 1.0);
            }
        }
    }

    let first_point = first_start + first_direction * first_parameter;
    let second_point = second_start + second_direction * second_parameter;
    first_point.distance(second_point)
}

fn body_collision_layers(
    rig: &HumanRig,
    poses: &BTreeMap<String, super::rig::HumanBonePose>,
    self_collision_enabled: bool,
    overlap_margin_m: f32,
) -> Result<BTreeMap<String, CollisionLayers>> {
    let names: Vec<_> = rig.body_names().map(str::to_owned).collect();
    ensure!(
        names.len() <= 31,
        "human ragdoll has {} bodies; collision layers reserve one of 32 bits for the environment",
        names.len()
    );

    let membership_bits: BTreeMap<_, _> = names
        .iter()
        .enumerate()
        .map(|(index, name)| (name.clone(), 1_u32 << (index + 1)))
        .collect();

    let mut excluded_pairs = BTreeSet::<(String, String)>::new();
    for child_name in &names {
        let child_bone = rig
            .bone(child_name)
            .ok_or_else(|| anyhow::anyhow!("human body '{child_name}' has no rig bone"))?;
        if let Some(parent_name) = nearest_body_parent(rig, child_bone.parent.as_deref()) {
            let pair = if parent_name < child_name.as_str() {
                (parent_name.to_owned(), child_name.clone())
            } else {
                (child_name.clone(), parent_name.to_owned())
            };
            excluded_pairs.insert(pair);
        }
    }

    if self_collision_enabled {
        let capsules: BTreeMap<_, _> = names
            .iter()
            .map(|name| {
                let pose = poses.get(name).ok_or_else(|| {
                    anyhow::anyhow!("human body '{name}' has no idle collision pose")
                })?;
                Ok((name.clone(), capsule_centerline(pose)))
            })
            .collect::<Result<_>>()?;

        for first_index in 0..names.len() {
            for second_index in (first_index + 1)..names.len() {
                let first_name = &names[first_index];
                let second_name = &names[second_index];
                let pair = (first_name.clone(), second_name.clone());
                if excluded_pairs.contains(&pair) {
                    continue;
                }

                let first = capsules[first_name];
                let second = capsules[second_name];
                let centerline_distance =
                    segment_segment_distance(first.start, first.end, second.start, second.end);
                let penetration = first.radius + second.radius - centerline_distance;
                if penetration > overlap_margin_m {
                    excluded_pairs.insert(pair);
                }
            }
        }
    } else {
        // Disabling ragdoll self-collision also excludes direct joint pairs.
        excluded_pairs.extend(names.iter().enumerate().flat_map(|(index, first)| {
            names
                .iter()
                .skip(index + 1)
                .map(move |second| (first.clone(), second.clone()))
        }));
    }

    // Bit zero is the Avian default environment layer. Every human collider keeps
    // that bit in its filter, while per-body membership bits permit selective self-contact.
    let mut layers = BTreeMap::new();
    for name in &names {
        let membership = membership_bits[name];
        let mut filters = 1_u32;
        if self_collision_enabled {
            for other_name in &names {
                if other_name == name {
                    continue;
                }
                let pair = if name < other_name {
                    (name.clone(), other_name.clone())
                } else {
                    (other_name.clone(), name.clone())
                };
                if !excluded_pairs.contains(&pair) {
                    filters |= membership_bits[other_name];
                }
            }
        }
        layers.insert(
            name.clone(),
            CollisionLayers::from_bits(membership, filters),
        );
    }
    Ok(layers)
}

pub fn spawn_human_player(
    mut commands: Commands,
    origin: Res<WorldOrigin>,
    rig: Option<Res<HumanRig>>,
    motions: Option<Res<HumanMotionLibrary>>,
    settings: Option<Res<HumanPhysicsSettings>>,
    asset_server: Res<AssetServer>,
    existing: Query<(), With<HumanPlayerRoot>>,
) {
    if !existing.is_empty() {
        return;
    }
    let (Some(rig), Some(motions), Some(settings)) = (rig, motions, settings) else {
        warn!("Human ragdoll not spawned: human data resources are unavailable");
        return;
    };
    let Some(idle) = motions.action("idle") else {
        warn!("Human ragdoll not spawned: no idle body-motion action");
        return;
    };

    let source_pose = match rig.sample_source_world_pose(idle, 0.0, false) {
        Ok(pose) => pose,
        Err(error) => {
            error!("Human idle pose could not be calibrated: {error:#}");
            return;
        }
    };
    let target_pose = match rig.sample_engine_body_pose(idle, 0.0, false) {
        Ok(pose) => pose,
        Err(error) => {
            error!("Human idle body pose could not be constructed: {error:#}");
            return;
        }
    };

    let rig_body_names: BTreeSet<_> = rig.body_names().map(str::to_owned).collect();
    let configured_body_names: BTreeSet<_> = settings.body_mass_fractions.keys().cloned().collect();
    if rig_body_names != configured_body_names {
        error!(
            "Human mass distribution tracks {:?} do not match rig body tracks {:?}",
            configured_body_names, rig_body_names
        );
        return;
    }

    let collision_layers = match body_collision_layers(
        &rig,
        &target_pose,
        settings.self_collision_enabled,
        settings.self_collision_overlap_margin_m,
    ) {
        Ok(layers) => layers,
        Err(error) => {
            error!("Human collision-layer setup failed: {error:#}");
            return;
        }
    };

    let mut bodies = BTreeMap::new();
    for name in rig.body_names() {
        let Some(pose) = target_pose.get(name) else {
            error!("Human body track '{name}' has no target pose");
            return;
        };
        let translation =
            (Vec3::from_array(pose.center_engine.to_array()).as_dvec3() - origin.0).as_vec3();
        let Some(mass_kg) = settings.body_mass_kg(name) else {
            error!("Human body '{name}' has no configured segment mass");
            return;
        };
        let collider_volume_m3 = pose.collision.volume_m3();
        let collider_density_kg_m3 = mass_kg / collider_volume_m3;
        if !collider_density_kg_m3.is_finite() || collider_density_kg_m3 <= 0.0 {
            error!("Human body '{name}' has invalid collider mass properties");
            return;
        }
        let collider = Collider::capsule(
            pose.collision.radius_m,
            pose.collision.capsule_segment_length_m,
        );
        let mut body_commands = commands.spawn((
            Name::new(format!("Human::{name}")),
            HumanBodyPart {
                bone_name: name.to_owned(),
            },
            RigidBody::Dynamic,
            collider,
            ColliderDensity(collider_density_kg_m3),
            collision_layers[name],
            Friction::new(settings.dynamic_friction)
                .with_static_coefficient(settings.static_friction),
            Restitution::new(settings.restitution),
            CollisionEventsEnabled,
            LinearVelocity(Vec3::ZERO),
            AngularVelocity(Vec3::ZERO),
            Transform {
                translation,
                rotation: pose.rotation_engine,
                ..default()
            },
        ));
        if name == "Root" {
            body_commands.insert(HumanPlayerRoot);
        }
        let entity = body_commands.id();
        bodies.insert(name.to_owned(), entity);
    }

    // Connect each segment to its nearest physical ancestor. Helper-bone marker tracks
    // supply the real shoulder, elbow and knee anchor locations.
    for child_name in rig.body_names().filter(|name| *name != "Root") {
        let Some(child_bone) = rig.bone(child_name) else {
            continue;
        };
        let Some(parent_name) = nearest_body_parent(&rig, child_bone.parent.as_deref()) else {
            error!("Human body '{child_name}' has no physical ancestor");
            return;
        };
        let (Some(&parent_entity), Some(&child_entity)) =
            (bodies.get(parent_name), bodies.get(child_name))
        else {
            error!("Human joint '{child_name}' is missing a body entity");
            return;
        };
        let marker_name = rig.joint_marker_for_body(child_name);
        let anchor_name = marker_name.unwrap_or(child_name);
        let Some(anchor_engine) = rig.joint_anchor_position_engine(&source_pose, anchor_name)
        else {
            error!("Human joint '{child_name}' has no convertible '{anchor_name}' anchor");
            return;
        };

        let parent_pose = &target_pose[parent_name];
        let child_pose = &target_pose[child_name];
        let parent_anchor_local =
            parent_pose.rotation_engine.inverse() * (anchor_engine - parent_pose.center_engine);
        let child_anchor_local =
            child_pose.rotation_engine.inverse() * (anchor_engine - child_pose.center_engine);
        // Both local joint frames represent the same world-space rest basis. This keeps
        // hinge/twist axes aligned when parent and child bones have different FBX rotations.
        let shared_joint_rotation = parent_pose.rotation_engine;
        let parent_joint_frame = local_joint_frame(
            parent_pose.rotation_engine,
            shared_joint_rotation,
            parent_anchor_local,
        );
        let child_joint_frame = local_joint_frame(
            child_pose.rotation_engine,
            shared_joint_rotation,
            child_anchor_local,
        );

        let damping = JointDamping {
            linear: settings.joint_damping_linear,
            angular: settings.joint_damping_angular,
        };

        if child_name.starts_with("Leg_") || child_name.starts_with("Hand_") {
            let limits = &settings.limits;
            let (min_angle, max_angle) = if child_name.starts_with("Leg_") {
                (limits.knee_min_rad, limits.knee_max_rad)
            } else {
                (limits.elbow_min_rad, limits.elbow_max_rad)
            };
            commands.spawn((
                Name::new(format!("HumanJoint::{parent_name}::{child_name}")),
                RevoluteJoint::new(parent_entity, child_entity)
                    .with_local_frame1(parent_joint_frame)
                    .with_local_frame2(child_joint_frame)
                    .with_hinge_axis(Vec3::X)
                    .with_angle_limits(min_angle, max_angle)
                    .with_point_compliance(settings.joint_point_compliance)
                    .with_align_compliance(settings.joint_angular_compliance)
                    .with_limit_compliance(settings.joint_angular_compliance),
                damping,
                JointCollisionDisabled,
                HumanJointConstraint,
            ));
        } else {
            let limits = &settings.limits;
            let (swing, twist) = if child_name.starts_with("Thigh_") {
                (limits.hip_swing_rad, limits.hip_twist_rad)
            } else if child_name.starts_with("Arm_") {
                (limits.shoulder_swing_rad, limits.shoulder_twist_rad)
            } else if child_name == "Head" {
                (limits.neck_swing_rad, limits.neck_twist_rad)
            } else {
                (limits.spine_swing_rad, limits.spine_twist_rad)
            };
            commands.spawn((
                Name::new(format!("HumanJoint::{parent_name}::{child_name}")),
                SphericalJoint::new(parent_entity, child_entity)
                    .with_local_frame1(parent_joint_frame)
                    .with_local_frame2(child_joint_frame)
                    .with_twist_axis(Vec3::Y)
                    .with_swing_limits(-swing, swing)
                    .with_twist_limits(-twist, twist)
                    .with_point_compliance(settings.joint_point_compliance)
                    .with_swing_compliance(settings.joint_angular_compliance)
                    .with_twist_compliance(settings.joint_angular_compliance),
                damping,
                JointCollisionDisabled,
                HumanJointConstraint,
            ));
        }
    }

    // The GLB is already exported in engine Y-up coordinates. Keep its model origin at
    // world zero while parenting it to the dynamic Root body at its physical center.
    let root_pose = &target_pose["Root"];
    let root_rotation_inverse = root_pose.rotation_engine.inverse();
    let root_center_relative = (root_pose.center_engine.as_dvec3() - origin.0).as_vec3();
    let visual_local_translation = root_rotation_inverse * -root_center_relative;
    let scene = asset_server.load(GltfAssetLabel::Scene(0).from_asset("human/Character_Rig.glb"));
    commands.entity(bodies["Root"]).with_children(|children| {
        children.spawn((
            Name::new("Human::SkinnedVisual"),
            HumanSkinnedSceneRoot,
            WorldAssetRoot(scene),
            Transform {
                translation: visual_local_translation,
                rotation: root_rotation_inverse,
                ..default()
            },
        ));
    });

    commands.insert_resource(HumanActionState::default());
    commands.insert_resource(HumanPlayer {
        root: bodies["Root"],
        bodies: bodies.clone(),
    });
    info!(
        "Spawned physics-driven human ragdoll with {} dynamic bodies and {} joints",
        bodies.len(),
        bodies.len().saturating_sub(1)
    );
}

fn local_joint_frame(
    body_rotation: Quat,
    shared_world_rotation: Quat,
    local_anchor: Vec3,
) -> Isometry3d {
    Isometry3d::new(
        local_anchor,
        (body_rotation.inverse() * shared_world_rotation).normalize(),
    )
}

/// Remove the player assembly when leaving Simulation so a newly selected world
/// receives a fresh dynamic ragdoll and action state.
pub fn despawn_human_player(
    mut commands: Commands,
    bodies: Query<Entity, With<HumanBodyPart>>,
    joints: Query<Entity, With<HumanJointConstraint>>,
) {
    for entity in &joints {
        commands.entity(entity).despawn();
    }
    for entity in &bodies {
        commands.entity(entity).despawn();
    }
    commands.remove_resource::<HumanPlayer>();
    commands.remove_resource::<HumanActionState>();
}

fn nearest_body_parent<'a>(rig: &'a HumanRig, mut parent: Option<&'a str>) -> Option<&'a str> {
    while let Some(name) = parent {
        if rig.body_names().any(|body| body == name) {
            return Some(name);
        }
        parent = rig.bone(name).and_then(|bone| bone.parent.as_deref());
    }
    None
}

#[cfg(test)]
mod tests {
    use super::{
        HumanPhysicsSettings, body_collision_layers, capsule_centerline, local_joint_frame,
        nearest_body_parent, segment_segment_distance,
    };
    use crate::human::{HumanMotionLibrary, HumanRig};
    use avian3d::prelude::CollisionLayers;
    use bevy::prelude::{KeyCode, Quat, Vec3};
    use std::{path::Path, time::Duration};

    #[test]
    fn human_physics_and_temporary_assists_load_from_manifest() {
        let settings = HumanPhysicsSettings::load(Path::new("data")).unwrap();
        assert!((settings.total_mass_kg - 73.0).abs() < f32::EPSILON);
        assert!((settings.body_mass_kg("Root").unwrap() - 8.1541).abs() < 1.0e-3);
        assert_eq!(settings.mass_distribution_reference, "de_leva_1996_male");
        assert_eq!(settings.controls.bindings.forward, KeyCode::KeyW);
        assert_eq!(settings.controls.bindings.backward, KeyCode::KeyS);
        assert_eq!(settings.controls.bindings.left, KeyCode::KeyA);
        assert_eq!(settings.controls.bindings.right, KeyCode::KeyD);
        assert_eq!(settings.controls.bindings.crouch, KeyCode::KeyC);
        assert_eq!(settings.controls.bindings.jump, KeyCode::Space);
        assert_eq!(
            settings.controls.bindings.run_modifiers,
            [KeyCode::ShiftLeft, KeyCode::ShiftRight]
        );
        assert_eq!(settings.body_mass_fractions.len(), 12);
        let mass_fraction_sum: f32 = settings.body_mass_fractions.values().sum();
        assert!((mass_fraction_sum - 1.0).abs() <= 1.0e-4);
        let body_mass_sum: f32 = settings
            .body_mass_fractions
            .keys()
            .filter_map(|name| settings.body_mass_kg(name))
            .sum();
        assert!((body_mass_sum - settings.total_mass_kg).abs() <= 1.0e-3);
        assert!(settings.temporary_assists.jump_launch.enabled);
        assert!(settings.temporary_assists.rotational_assistance.enabled);
        let climb_capacity_n = settings.dynamic_friction * settings.climb_contact_push_force_n;
        let body_weight_n = settings.total_mass_kg * 9.81;
        assert!(
            climb_capacity_n >= body_weight_n * super::MIN_CLIMB_SUPPORT_FACTOR,
            "climb friction capacity {climb_capacity_n:.1} N is below body weight {body_weight_n:.1} N"
        );
        assert_eq!(settings.temporary_assists.jump_launch.kind, "impulse");
        assert_eq!(
            settings.temporary_assists.rotational_assistance.kind,
            "bounded_torque_pd"
        );
    }

    #[test]
    fn idle_capsule_filters_preserve_environment_contacts_and_skip_initial_overlaps() {
        let rig = HumanRig::load(Path::new("data")).unwrap();
        let motions = HumanMotionLibrary::load(Path::new("data")).unwrap();
        let idle = motions.action("idle").unwrap();
        let poses = rig.sample_engine_body_pose(idle, 0.0, false).unwrap();
        let layers = body_collision_layers(&rig, &poses, true, 0.005).unwrap();

        let names: Vec<_> = rig.body_names().collect();
        let mut excluded_overlaps = 0;
        let mut enabled_separated_pairs = 0;
        for first_index in 0..names.len() {
            for second_index in (first_index + 1)..names.len() {
                let first_name = names[first_index];
                let second_name = names[second_index];
                let first_pose = &poses[first_name];
                let second_pose = &poses[second_name];
                let first = capsule_centerline(first_pose);
                let second = capsule_centerline(second_pose);
                let distance =
                    segment_segment_distance(first.start, first.end, second.start, second.end);
                let initially_overlapping = first.radius + second.radius - distance > 0.005;
                let can_collide = layers[first_name].interacts_with(layers[second_name]);
                assert_eq!(
                    can_collide,
                    !initially_overlapping
                        && nearest_body_parent(
                            &rig,
                            rig.bone(first_name).unwrap().parent.as_deref()
                        ) != Some(second_name)
                        && nearest_body_parent(
                            &rig,
                            rig.bone(second_name).unwrap().parent.as_deref()
                        ) != Some(first_name),
                    "unexpected idle self-collision policy for {first_name} / {second_name}"
                );
                if initially_overlapping {
                    excluded_overlaps += 1;
                } else if can_collide {
                    enabled_separated_pairs += 1;
                }
            }

            let body_layer = layers[names[first_index]];
            assert!(
                body_layer.interacts_with(CollisionLayers::DEFAULT)
                    && CollisionLayers::DEFAULT.interacts_with(body_layer),
                "human body '{}' must keep collision with the environment",
                names[first_index]
            );
        }
        assert!(excluded_overlaps > 0);
        assert!(enabled_separated_pairs > 0);
    }

    #[test]
    fn capsule_segment_distance_handles_degenerate_and_skew_segments() {
        let degenerate_distance =
            segment_segment_distance(Vec3::ZERO, Vec3::ZERO, Vec3::X, Vec3::X);
        assert!((degenerate_distance - 1.0).abs() < f32::EPSILON);

        let skew_distance = segment_segment_distance(
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, -1.0),
            Vec3::new(0.0, 1.0, 1.0),
        );
        assert!((skew_distance - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn joint_frames_reconstruct_the_same_world_rest_basis() {
        let parent_rotation = Quat::from_rotation_y(0.63) * Quat::from_rotation_z(-0.18);
        let child_rotation = Quat::from_rotation_x(-0.91) * Quat::from_rotation_z(0.27);
        let parent_frame =
            local_joint_frame(parent_rotation, parent_rotation, Vec3::new(0.1, -0.2, 0.3));
        let child_frame =
            local_joint_frame(child_rotation, parent_rotation, Vec3::new(-0.4, 0.2, 0.1));
        let parent_world_basis = parent_rotation * parent_frame.rotation;
        let child_world_basis = child_rotation * child_frame.rotation;
        assert!(parent_world_basis.dot(child_world_basis).abs() > 0.99999);
    }

    #[test]
    fn spherical_joint_preserves_world_anchor_under_body_motion() {
        use avian3d::prelude::{
            AngularVelocity, Collider, Gravity, JointCollisionDisabled, JointDamping,
            LinearVelocity, Mass, PhysicsPlugins, RigidBody, SphericalJoint, SubstepCount,
        };
        use bevy::{asset::AssetPlugin, mesh::MeshPlugin, prelude::*, time::TimeUpdateStrategy};

        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            PhysicsPlugins::default(),
            AssetPlugin::default(),
            MeshPlugin,
        ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 120.0,
        )))
        .insert_resource(Time::<Fixed>::from_hz(120.0))
        .insert_resource(SubstepCount(16))
        .insert_resource(Gravity::ZERO);
        app.finish();

        let parent_rotation = Quat::from_rotation_y(0.42) * Quat::from_rotation_z(0.16);
        let child_rotation = Quat::from_rotation_x(-0.71) * Quat::from_rotation_z(0.28);
        let parent_translation = Vec3::new(-0.3, 0.0, 0.1);
        let child_translation = Vec3::new(0.4, 1.0, -0.2);
        let anchor_world = Vec3::new(0.05, 0.48, -0.04);
        let parent_anchor_local = parent_rotation.inverse() * (anchor_world - parent_translation);
        let child_anchor_local = child_rotation.inverse() * (anchor_world - child_translation);
        let shared_joint_rotation = parent_rotation;
        let parent_frame =
            local_joint_frame(parent_rotation, shared_joint_rotation, parent_anchor_local);
        let child_frame =
            local_joint_frame(child_rotation, shared_joint_rotation, child_anchor_local);

        let parent = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::capsule(0.15, 0.35),
                Mass(3.0),
                LinearVelocity(Vec3::ZERO),
                AngularVelocity(Vec3::ZERO),
                Transform {
                    translation: parent_translation,
                    rotation: parent_rotation,
                    ..default()
                },
            ))
            .id();
        let child = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::capsule(0.15, 0.35),
                Mass(2.0),
                LinearVelocity(Vec3::new(1.5, -0.2, 0.6)),
                AngularVelocity(Vec3::ZERO),
                Transform {
                    translation: child_translation,
                    rotation: child_rotation,
                    ..default()
                },
            ))
            .id();

        app.world_mut().spawn((
            SphericalJoint::new(parent, child)
                .with_local_frame1(parent_frame)
                .with_local_frame2(child_frame)
                .with_twist_axis(Vec3::Y)
                .with_swing_limits(-0.8, 0.8)
                .with_twist_limits(-0.6, 0.6)
                .with_point_compliance(1.0e-7)
                .with_swing_compliance(1.0e-6)
                .with_twist_compliance(1.0e-6),
            JointDamping {
                linear: 2.0,
                angular: 2.0,
            },
            JointCollisionDisabled,
        ));

        for _ in 0..240 {
            app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
                1.0 / 120.0,
            )));
            app.update();
        }

        let parent_transform = app.world().get::<Transform>(parent).unwrap();
        let child_transform = app.world().get::<Transform>(child).unwrap();
        let parent_anchor_world =
            parent_transform.translation + parent_transform.rotation * parent_anchor_local;
        let child_anchor_world =
            child_transform.translation + child_transform.rotation * child_anchor_local;
        let anchor_error = parent_anchor_world.distance(child_anchor_world);
        assert!(
            anchor_error < 0.02,
            "Avian solver allowed {:.4} m of spherical-joint anchor drift",
            anchor_error
        );
    }
}

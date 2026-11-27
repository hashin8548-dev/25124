use std::{collections::BTreeMap, fs, path::Path};

use anyhow::{Context, Result, ensure};
use avian3d::prelude::{
    AngularVelocity, Collider, ColliderDensity, CollisionEventsEnabled, Friction,
    JointCollisionDisabled, JointDamping, LinearVelocity, Restitution, RevoluteJoint,
    RigidBody, SphericalJoint,
};
use bevy::{gltf::prelude::GltfAssetLabel, prelude::*};
use serde::Deserialize;

use crate::{config::RuntimeConfig, world::WorldOrigin};

use super::{HumanMotionLibrary, HumanRig};

#[derive(Clone, Debug, Resource)]
pub struct HumanPhysicsSettings {
    pub density_kg_m3: f32,
    pub static_friction: f32,
    pub dynamic_friction: f32,
    pub restitution: f32,
    pub joint_damping_linear: f32,
    pub joint_damping_angular: f32,
    pub joint_point_compliance: f32,
    pub joint_angular_compliance: f32,
    pub limits: HumanJointLimits,
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
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct HumanPhysicsSettingsFile {
    density_kg_m3: f32,
    static_friction: f32,
    dynamic_friction: f32,
    restitution: f32,
    joint_damping_linear: f32,
    joint_damping_angular: f32,
    joint_point_compliance: f32,
    joint_angular_compliance: f32,
    limits: HumanJointLimits,
}

#[derive(Component, Clone, Debug)]
pub struct HumanBodyPart {
    pub bone_name: String,
}

#[derive(Component, Clone, Copy, Debug)]
pub struct HumanPlayerRoot;

#[derive(Clone, Debug, Resource)]
pub struct HumanPlayer {
    pub root: Entity,
    pub bodies: BTreeMap<String, Entity>,
}

impl HumanPhysicsSettings {
    pub fn load(data_dir: &Path) -> Result<Self> {
        let path = data_dir.join("human").join("human.json");
        let text = fs::read_to_string(&path)
            .with_context(|| format!("reading human physics settings {}", path.display()))?;
        let manifest: HumanManifest = serde_json::from_str(&text)
            .with_context(|| format!("parsing human physics settings {}", path.display()))?;
        ensure!(manifest.schema_version == 1, "unsupported human manifest schema version");
        let source = manifest.physics;
        ensure!(
            source.density_kg_m3.is_finite() && source.density_kg_m3 > 0.0,
            "human body density must be positive and finite"
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
            ensure!(value.is_finite() && (0.0..=std::f32::consts::PI).contains(&value), "human {name} limit is invalid");
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
            density_kg_m3: source.density_kg_m3,
            static_friction: source.static_friction,
            dynamic_friction: source.dynamic_friction,
            restitution: source.restitution,
            joint_damping_linear: source.joint_damping_linear,
            joint_damping_angular: source.joint_damping_angular,
            joint_point_compliance: source.joint_point_compliance,
            joint_angular_compliance: source.joint_angular_compliance,
            limits: source.limits,
        })
    }
}

pub fn spawn_human_player(
    mut commands: Commands,
    config: Res<RuntimeConfig>,
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

    let mut bodies = BTreeMap::new();
    for name in rig.body_names() {
        let Some(pose) = target_pose.get(name) else {
            error!("Human body track '{name}' has no target pose");
            return;
        };
        let translation = (
            Vec3::from_array(pose.center_engine.to_array())
                .as_dvec3()
                - origin.0
        )
        .as_vec3();
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
                ColliderDensity(settings.density_kg_m3),
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
        let marker_name = joint_marker_for(child_name);
        let anchor_name = marker_name.unwrap_or(child_name);
        let Some(anchor_source) = source_pose.get(anchor_name) else {
            error!("Human joint '{child_name}' has no '{anchor_name}' marker transform");
            return;
        };
        let source_anchor = anchor_source.transform_point3(Vec3::ZERO);
        let source_to_engine = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let anchor_engine = source_to_engine * source_anchor;

        let parent_pose = &target_pose[parent_name];
        let child_pose = &target_pose[child_name];
        let parent_anchor_local =
            parent_pose.rotation_engine.inverse() * (anchor_engine - parent_pose.center_engine);
        let child_anchor_local =
            child_pose.rotation_engine.inverse() * (anchor_engine - child_pose.center_engine);

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
            let world_hinge_axis = parent_pose.rotation_engine * Vec3::X;
            let parent_local_axis =
                (parent_pose.rotation_engine.inverse() * world_hinge_axis).normalize_or_zero();
            commands.spawn((
                Name::new(format!("HumanJoint::{parent_name}::{child_name}")),
                RevoluteJoint::new(parent_entity, child_entity)
                    .with_local_anchor1(parent_anchor_local)
                    .with_local_anchor2(child_anchor_local)
                    .with_hinge_axis(parent_local_axis)
                    .with_angle_limits(min_angle, max_angle)
                    .with_point_compliance(settings.joint_point_compliance)
                    .with_align_compliance(settings.joint_angular_compliance)
                    .with_limit_compliance(settings.joint_angular_compliance),
                damping,
                JointCollisionDisabled,
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
            let local_twist_axis = parent_pose.rotation_engine.inverse()
                * (parent_pose.rotation_engine * Vec3::Y);
            commands.spawn((
                Name::new(format!("HumanJoint::{parent_name}::{child_name}")),
                SphericalJoint::new(parent_entity, child_entity)
                    .with_local_anchor1(parent_anchor_local)
                    .with_local_anchor2(child_anchor_local)
                    .with_twist_axis(local_twist_axis.normalize_or_zero())
                    .with_swing_limits(-swing, swing)
                    .with_twist_limits(-twist, twist)
                    .with_point_compliance(settings.joint_point_compliance)
                    .with_swing_compliance(settings.joint_angular_compliance)
                    .with_twist_compliance(settings.joint_angular_compliance),
                damping,
                JointCollisionDisabled,
            ));
        }
    }

    // The GLB is already exported in engine Y-up coordinates. Keep its model origin at
    // world zero while parenting it to the dynamic Root body at its physical center.
    let root_pose = &target_pose["Root"];
    let root_rotation_inverse = root_pose.rotation_engine.inverse();
    let visual_local_translation = root_rotation_inverse * -root_pose.center_engine;
    let scene = asset_server.load(GltfAssetLabel::Scene(0).from_asset("human/Character_Rig.glb"));
    commands.entity(bodies["Root"]).with_children(|children| {
        children.spawn((
            Name::new("Human::SkinnedVisual"),
            WorldAssetRoot(scene),
            Transform {
                translation: visual_local_translation,
                rotation: root_rotation_inverse,
                ..default()
            },
        ));
    });

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

fn nearest_body_parent<'a>(rig: &'a HumanRig, mut parent: Option<&'a str>) -> Option<&'a str> {
    while let Some(name) = parent {
        if rig.body_names().any(|body| body == name) {
            return Some(name);
        }
        parent = rig.bone(name).and_then(|bone| bone.parent.as_deref());
    }
    None
}

fn joint_marker_for(child: &str) -> Option<&'static str> {
    match child {
        "Arm_L" => Some("Shoulder_L"),
        "Arm_R" => Some("Shoulder_R"),
        "Hand_L" => Some("Elbow_L"),
        "Hand_R" => Some("Elbow_R"),
        "Leg_L" => Some("Knee_L"),
        "Leg_R" => Some("Knee_R"),
        _ => None,
    }
}

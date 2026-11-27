use avian3d::prelude::{AngularVelocity, Forces, LinearVelocity};
use bevy::prelude::*;

use super::{HumanBodyPart, HumanMotionLibrary, HumanPhysicsSettings, HumanPlayer, HumanRig};

/// Current source action selected by the gameplay input state machine.
#[derive(Clone, Debug, Resource)]
pub struct HumanActionState {
    pub action_name: String,
    pub elapsed_seconds: f32,
    pub reverse: bool,
    pub heading_yaw_radians: f32,
    /// One-shot launch request produced by the render-rate input system.
    pub jump_launch_pending: bool,
    /// Continuous wall-grip intent, consumed on every fixed physics step.
    pub wall_grip_active: bool,
    /// Outward normal returned by the most recent wall probe.
    pub wall_grip_normal: Vec3,
    /// Physical root position captured at the start of the selected action.
    pub root_motion_origin_world: Option<Vec3>,
    /// Root body centre in the action's first (or reverse-playback last) sample.
    pub root_motion_reference_center_engine: Option<Vec3>,
}

impl Default for HumanActionState {
    fn default() -> Self {
        Self {
            action_name: "idle".to_owned(),
            elapsed_seconds: 0.0,
            reverse: false,
            heading_yaw_radians: 0.0,
            jump_launch_pending: false,
            wall_grip_active: false,
            wall_grip_normal: Vec3::ZERO,
            root_motion_origin_world: None,
            root_motion_reference_center_engine: None,
        }
    }
}

impl HumanActionState {
    /// Change the sampled action without mutating any source JSON data.
    pub fn select(&mut self, action_name: &str, reverse: bool) {
        if self.action_name != action_name || self.reverse != reverse {
            self.action_name = action_name.to_owned();
            self.elapsed_seconds = 0.0;
            self.reverse = reverse;
            self.root_motion_origin_world = None;
            self.root_motion_reference_center_engine = None;
        }
    }
}

/// Applies bounded PD torques to physical body segments using the sampled action poses.
/// It does not animate a skeleton or inject root velocity; contacts and joints determine
/// the resulting translation.
pub fn drive_human_action_pose(
    time: Res<Time<Fixed>>,
    mut action_state: Option<ResMut<HumanActionState>>,
    motions: Option<Res<HumanMotionLibrary>>,
    rig: Option<Res<HumanRig>>,
    settings: Option<Res<HumanPhysicsSettings>>,
    player: Option<Res<HumanPlayer>>,
    root_transforms: Query<&Transform>,
    mut bodies: Query<(
        &HumanBodyPart,
        &Transform,
        &LinearVelocity,
        &AngularVelocity,
        Forces,
    )>,
) {
    let (Some(mut action_state), Some(motions), Some(rig), Some(settings), Some(player)) =
        (action_state, motions, rig, settings, player)
    else {
        return;
    };

    let Some(action) = motions.action(&action_state.action_name) else {
        warn!(
            "Human action '{}' is missing; selecting idle",
            action_state.action_name
        );
        action_state.select("idle", false);
        return;
    };
    let Ok(root_transform) = root_transforms.get(player.root) else {
        return;
    };

    if action_state.root_motion_origin_world.is_none()
        || action_state.root_motion_reference_center_engine.is_none()
    {
        let reference_pose = match rig.sample_engine_body_pose(action, 0.0, action_state.reverse) {
            Ok(pose) => pose,
            Err(error) => {
                warn!("Human Root reference sampling failed: {error:#}");
                return;
            }
        };
        let Some(reference_root) = reference_pose.get("Root") else {
            return;
        };
        action_state.root_motion_origin_world = Some(root_transform.translation);
        action_state.root_motion_reference_center_engine = Some(reference_root.center_engine);
    }

    action_state.elapsed_seconds += time.delta_secs();
    let sampled = match rig.sample_engine_body_pose(
        action,
        action_state.elapsed_seconds,
        action_state.reverse,
    ) {
        Ok(pose) => pose,
        Err(error) => {
            warn!("Human action target sampling failed: {error:#}");
            return;
        }
    };
    let Some(root_target) = sampled.get("Root") else {
        return;
    };

    let (root_yaw, _, _) = root_transform.rotation.to_euler(EulerRot::YXZ);
    let heading_yaw = if action_state.heading_yaw_radians.is_finite() {
        action_state.heading_yaw_radians
    } else {
        root_yaw
    };
    let heading = Quat::from_rotation_y(heading_yaw);
    let desired_root_rotation = heading * root_target.rotation_engine;
    let rotational_assist = &settings.temporary_assists.rotational_assistance;
    let jump_launch_assist = &settings.temporary_assists.jump_launch;
    let suppress_authored_jump_root =
        action_state.action_name == "jump" && jump_launch_assist.enabled;
    let authored_root_target = match (
        action_state.root_motion_origin_world,
        action_state.root_motion_reference_center_engine,
    ) {
        (Some(origin_world), Some(reference_center)) if !suppress_authored_jump_root => {
            Some(origin_world + heading * (root_target.center_engine - reference_center))
        }
        _ => None,
    };
    let jump_impulse =
        if action_state.jump_launch_pending && settings.temporary_assists.jump_launch.enabled {
            rig.bone("Root")
                .and_then(|bone| bone.collision_geometry.as_ref())
                .map(|geometry| {
                    let radius = geometry.radius_m;
                    let segment = geometry.capsule_segment_length_m;
                    let volume = std::f32::consts::PI * radius * radius * segment
                        + (4.0 / 3.0) * std::f32::consts::PI * radius.powi(3);
                    let estimated_root_mass = volume * settings.density_kg_m3;
                    Vec3::Y
                        * estimated_root_mass
                        * settings.temporary_assists.jump_launch.launch_speed_m_s
                })
        } else {
            None
        };
    let wall_grip_force = if action_state.wall_grip_active
        && action_state.wall_grip_normal.is_finite()
        && action_state.wall_grip_normal.length_squared() > 0.9
    {
        Some(-action_state.wall_grip_normal.normalize() * settings.climb_contact_push_force_n * 0.5)
    } else {
        None
    };

    for (part, transform, linear_velocity, angular_velocity, mut forces) in &mut bodies {
        // Apply one-shot and continuous assistance at the same fixed rate as Avian's solver.
        if part.bone_name == "Root" {
            if let Some(desired_root_center) = authored_root_target {
                let root_force = ((desired_root_center - transform.translation)
                    * settings.pose_position_stiffness_n_per_m
                    - linear_velocity.0 * settings.pose_position_damping_n_s_per_m)
                    .clamp_length_max(settings.max_pose_force_n);
                forces.apply_force(root_force);
            }
            if let Some(impulse) = jump_impulse {
                forces.apply_linear_impulse(impulse);
            }
        } else if (part.bone_name == "Hand_L" || part.bone_name == "Hand_R")
            && let Some(force) = wall_grip_force
        {
            forces.apply_force(force);
        }

        let desired_rotation = if part.bone_name == "Root" {
            if !rotational_assist.enabled {
                continue;
            }
            desired_root_rotation
        } else {
            let Some(target) = sampled.get(&part.bone_name) else {
                continue;
            };
            let root_relative = root_target.rotation_engine.inverse() * target.rotation_engine;
            heading * root_relative
        };

        let mut rotation_error = (desired_rotation * transform.rotation.inverse()).normalize();
        if rotation_error.w < 0.0 {
            rotation_error = -rotation_error;
        }
        let (axis, angle) = rotation_error.to_axis_angle();
        if !angle.is_finite() || !axis.is_finite() {
            continue;
        }

        let (stiffness, damping, max_torque) = if part.bone_name == "Root" {
            (
                settings.pose_stiffness_nm_per_rad * rotational_assist.strength,
                settings.pose_damping_nm_s_per_rad * rotational_assist.strength,
                rotational_assist.max_torque_nm,
            )
        } else {
            (
                settings.pose_stiffness_nm_per_rad,
                settings.pose_damping_nm_s_per_rad,
                settings.max_pose_torque_nm,
            )
        };

        // Translate only non-root segments toward the action's calibrated body targets.
        // Root translation is never commanded: equal-and-opposite joint/contact reactions
        // determine how the character moves over the ground.
        if part.bone_name != "Root" {
            if let Some(target) = sampled.get(&part.bone_name) {
                let root_relative = root_target.rotation_engine.inverse()
                    * (target.center_engine - root_target.center_engine);
                let desired_center = root_transform.translation + heading * root_relative;
                let force = ((desired_center - transform.translation)
                    * settings.pose_position_stiffness_n_per_m
                    - linear_velocity.0 * settings.pose_position_damping_n_s_per_m)
                    .clamp_length_max(settings.max_pose_force_n);
                forces.apply_force(force);
            }
        }

        let torque = (axis * (stiffness * angle) - angular_velocity.0 * damping)
            .clamp_length_max(max_torque);
        forces.apply_torque(torque);
    }

    // A jump launch is consumed exactly once, even if multiple physics substeps run.
    action_state.jump_launch_pending = false;
}

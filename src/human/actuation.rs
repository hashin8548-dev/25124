use avian3d::prelude::{AngularVelocity, Forces, LinearVelocity, WriteRigidBodyForces};
use bevy::{math::DVec3, prelude::*};

use crate::world::WorldOrigin;

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
    /// Absolute world-space root position captured at the start of the selected action.
    /// DVec3 keeps this reference invariant when the simulation's floating origin is rebased.
    pub root_motion_origin_world: Option<DVec3>,
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

/// Applies bounded physical forces and torques to pursue sampled action poses.
/// There is no fixed-speed root translation: the root follows only measured source-track
/// offsets through a bounded force, and meaningful horizontal displacement remains contact-driven.
#[allow(clippy::too_many_arguments)]
pub fn drive_human_action_pose(
    time: Res<Time<Fixed>>,
    action_state: Option<ResMut<HumanActionState>>,
    motions: Option<Res<HumanMotionLibrary>>,
    rig: Option<Res<HumanRig>>,
    settings: Option<Res<HumanPhysicsSettings>>,
    player: Option<Res<HumanPlayer>>,
    origin: Res<WorldOrigin>,
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
        action_state.root_motion_origin_world =
            Some(root_transform.translation.as_dvec3() + origin.0);
        action_state.root_motion_reference_center_engine = Some(reference_root.center_engine);
    }

    let sample_time = if action.looping {
        action_state.elapsed_seconds += time.delta_secs();
        action_state.elapsed_seconds
    } else {
        action_state.elapsed_seconds =
            (action_state.elapsed_seconds + time.delta_secs()).min(action.duration_seconds);
        action_state.elapsed_seconds
    };
    let sampled = match rig.sample_engine_body_pose(action, sample_time, action_state.reverse) {
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
        action_state.action_name == settings.controls.jump && jump_launch_assist.enabled;
    let root_translation_is_contact_driven = matches!(
        action_state.action_name.as_str(),
        action if action == settings.controls.climb || action == settings.controls.fall
    );
    let authored_root_target = match (
        action_state.root_motion_origin_world,
        action_state.root_motion_reference_center_engine,
    ) {
        (Some(origin_absolute), Some(reference_center))
            if !suppress_authored_jump_root && !root_translation_is_contact_driven =>
        {
            Some(root_motion_target_local(
                origin_absolute,
                origin.0,
                heading,
                root_target.center_engine - reference_center,
            ))
        }
        _ => None,
    };
    let jump_launch_enabled =
        action_state.jump_launch_pending && settings.temporary_assists.jump_launch.enabled;
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
                let root_force = root_vertical_pose_force(
                    desired_root_center.y,
                    transform.translation.y,
                    linear_velocity.0.y,
                    settings.pose_position_stiffness_n_per_m,
                    settings.pose_position_damping_n_s_per_m,
                    settings.max_pose_force_n,
                );
                forces.apply_force(root_force);
            }
        } else if (part.bone_name == "Hand_L" || part.bone_name == "Hand_R")
            && let Some(force) = wall_grip_force
        {
            forces.apply_force(force);
        }

        // Apply the launch assist proportionally to every segment's mass. Equal delta-v
        // preserves the ragdoll's internal rest motion while giving its centre of mass
        // the configured launch speed without a single-body impulse shock.
        if jump_launch_enabled
            && let Some(segment_mass) = settings.body_mass_kg(&part.bone_name)
        {
            let impulse = launch_impulse_for_mass(
                segment_mass,
                settings.temporary_assists.jump_launch.launch_speed_m_s,
            );
            forces.apply_linear_impulse(impulse);
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

        // Translate non-root segments toward the action's calibrated body targets.
        // The Root's authored trajectory is handled separately above as a bounded force.
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

fn launch_impulse_for_mass(segment_mass_kg: f32, launch_speed_m_s: f32) -> Vec3 {
    Vec3::Y * segment_mass_kg * launch_speed_m_s
}

fn root_motion_target_local(
    origin_absolute: DVec3,
    world_origin: DVec3,
    heading: Quat,
    source_root_offset: Vec3,
) -> Vec3 {
    (origin_absolute - world_origin).as_vec3() + heading * source_root_offset
}

fn root_vertical_pose_force(
    desired_y: f32,
    current_y: f32,
    vertical_velocity: f32,
    stiffness: f32,
    damping: f32,
    max_force: f32,
) -> Vec3 {
    let force_y = ((desired_y - current_y) * stiffness - vertical_velocity * damping)
        .clamp(-max_force, max_force);
    Vec3::Y * force_y
}

#[cfg(test)]
mod tests {
    use super::{
        launch_impulse_for_mass, root_motion_target_local, root_vertical_pose_force,
    };
    use bevy::{math::DVec3, prelude::{Quat, Vec3}};

    #[test]
    fn authored_root_position_assistance_never_resists_horizontal_motion() {
        let force = root_vertical_pose_force(2.0, 1.0, 0.5, 180.0, 28.0, 240.0);
        assert_eq!(force.x, 0.0);
        assert_eq!(force.z, 0.0);
        assert!((force.y - 166.0).abs() < f32::EPSILON);
    }

    #[test]
    fn root_action_target_is_invariant_under_floating_origin_rebase() {
        let absolute_start = DVec3::new(180_000.0, 8_500.0, -92_000.0);
        let before_rebase = DVec3::new(179_968.0, 8_192.0, -92_032.0);
        let after_rebase = before_rebase + DVec3::new(32.0, 256.0, -64.0);
        let heading = Quat::from_rotation_y(0.7);
        let source_offset = Vec3::new(0.03, 0.12, -0.02);

        let target_before =
            root_motion_target_local(absolute_start, before_rebase, heading, source_offset);
        let target_after =
            root_motion_target_local(absolute_start, after_rebase, heading, source_offset);
        let expected_shift = (after_rebase - before_rebase).as_vec3();

        assert!(
            (target_after - (target_before - expected_shift)).length() < 1.0e-4,
            "the local action target must shift by the same vector as the simulation origin"
        );
        let absolute_target_before = target_before.as_dvec3() + before_rebase;
        let absolute_target_after = target_after.as_dvec3() + after_rebase;
        assert!(
            (absolute_target_before - absolute_target_after).length() < 1.0e-3,
            "rebasing must preserve the absolute action target"
        );
    }

    #[test]
    fn jump_assist_gives_every_segment_equal_delta_velocity() {
        let settings = crate::human::HumanPhysicsSettings::load(std::path::Path::new("data"))
            .unwrap();
        let launch_speed = 4.5;
        let mut total_impulse = Vec3::ZERO;
        for name in settings.body_mass_fractions.keys() {
            let mass = settings.body_mass_kg(name).unwrap();
            let impulse = launch_impulse_for_mass(mass, launch_speed);
            total_impulse += impulse;
            assert!(
                ((impulse / mass).y - launch_speed).abs() < 1.0e-5,
                "segment '{name}' should receive the same launch delta velocity"
            );
        }
        assert!(
            (total_impulse - Vec3::Y * settings.total_mass_kg * launch_speed).length()
                < 1.0e-3,
            "sum of segment impulses must equal whole-body launch momentum"
        );
    }
}

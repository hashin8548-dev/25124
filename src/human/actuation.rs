use avian3d::prelude::{AngularVelocity, Forces};
use bevy::prelude::*;

use super::{HumanBodyPart, HumanMotionLibrary, HumanPhysicsSettings, HumanPlayer, HumanRig};

/// Current source action selected by the gameplay input state machine.
#[derive(Clone, Debug, Resource)]
pub struct HumanActionState {
    pub action_name: String,
    pub elapsed_seconds: f32,
    pub reverse: bool,
}

impl Default for HumanActionState {
    fn default() -> Self {
        Self {
            action_name: "idle".to_owned(),
            elapsed_seconds: 0.0,
            reverse: false,
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
        }
    }
}

/// Applies bounded PD torques to physical body segments using the sampled action poses.
/// It does not animate a skeleton or inject root velocity; contacts and joints determine
/// the resulting translation.
pub fn drive_human_action_pose(
    time: Res<Time<Fixed>>,
    mut action_state: ResMut<HumanActionState>,
    motions: Res<HumanMotionLibrary>,
    rig: Res<HumanRig>,
    settings: Res<HumanPhysicsSettings>,
    player: Res<HumanPlayer>,
    root_transform: Query<&Transform>,
    mut bodies: Query<(&HumanBodyPart, &Transform, &AngularVelocity, Forces)>,
) {
    let Some(action) = motions.action(&action_state.action_name) else {
        warn!(
            "Human action '{}' is missing; selecting idle",
            action_state.action_name
        );
        action_state.select("idle", false);
        return;
    };
    let Ok(root_transform) = root_transform.get(player.root) else {
        return;
    };

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

    let (heading_yaw, _, _) = root_transform.rotation.to_euler(EulerRot::YXZ);
    let heading = Quat::from_rotation_y(heading_yaw);
    let desired_root_rotation = heading * root_target.rotation_engine;
    let rotational_assist = &settings.temporary_assists.rotational_assistance;

    for (part, transform, angular_velocity, mut forces) in &mut bodies {
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

        let torque = (axis * (stiffness * angle) - angular_velocity.0 * damping)
            .clamp_length_max(max_torque);
        forces.apply_torque(torque);
    }
}

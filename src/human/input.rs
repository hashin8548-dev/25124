use avian3d::prelude::{Dir3, LinearVelocity, SpatialQuery, SpatialQueryFilter};
use bevy::prelude::*;

use super::{
    HumanActionState, HumanMotionLibrary, HumanPhysicsSettings, HumanPlayer, HumanPlayerRoot,
};

const WALL_PROBE_DISTANCE_M: f32 = 1.0;
const GROUND_PROBE_DISTANCE_M: f32 = 1.4;
const DESCENDING_THRESHOLD_M_S: f32 = 0.25;

/// Selects source motion targets from player input and physical environment probes.
/// The only launch impulse is the explicit, removable test assist in human.json.
pub fn update_human_controls(
    keys: Res<ButtonInput<KeyCode>>,
    spatial_query: SpatialQuery,
    mut action_state: Option<ResMut<HumanActionState>>,
    motions: Option<Res<HumanMotionLibrary>>,
    settings: Option<Res<HumanPhysicsSettings>>,
    player: Option<Res<HumanPlayer>>,
    root: Query<(&Transform, &LinearVelocity), With<HumanPlayerRoot>>,
) {
    let (Some(mut action_state), Some(motions), Some(settings), Some(player)) =
        (action_state, motions, settings, player)
    else {
        return;
    };
    let Ok((root_transform, root_velocity)) = root.single() else {
        return;
    };

    let forward = (root_transform.rotation * Vec3::NEG_Z)
        .with_y(0.0)
        .normalize_or_zero();
    if forward.length_squared() < 0.9 {
        return;
    }

    // Ignore all ragdoll bodies so probes only report the environment.
    let filter = SpatialQueryFilter::from_excluded_entities(player.bodies.values().copied());
    let chest_origin = root_transform.translation + Vec3::Y * 0.75;
    let wall_hit = Dir3::new(forward).ok().and_then(|direction| {
        spatial_query.cast_ray(
            chest_origin,
            direction,
            WALL_PROBE_DISTANCE_M,
            true,
            &filter,
        )
    });
    let ground_hit = spatial_query.cast_ray(
        root_transform.translation + Vec3::Y * 0.25,
        Dir3::NEG_Y,
        GROUND_PROBE_DISTANCE_M,
        true,
        &filter,
    );
    let grounded = ground_hit.is_some_and(|hit| hit.normal.dot(Vec3::Y) > 0.55);
    let descending = root_velocity.0.y < -DESCENDING_THRESHOLD_M_S;

    let key_w = keys.pressed(KeyCode::KeyW);
    let key_a = keys.pressed(KeyCode::KeyA);
    let key_s = keys.pressed(KeyCode::KeyS);
    let key_d = keys.pressed(KeyCode::KeyD);
    let crouching = keys.pressed(KeyCode::KeyC);
    let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let jump_pressed = keys.just_pressed(KeyCode::Space);
    let jump_held = keys.pressed(KeyCode::Space);
    let horizontal_input = key_w || key_a || key_s || key_d;
    let backwards = key_s && !key_w;
    let run_input = key_w || key_a || key_d;
    let wall_ahead = wall_hit.is_some();
    let was_climbing = action_state.action_name == settings.controls.climb;

    let (selected, launch_jump, apply_wall_grip) = if crouching {
        if horizontal_input {
            (
                Some((
                    settings.controls.crouch.as_str(),
                    backwards && settings.controls.crouch_backwards_reverse,
                )),
                false,
                false,
            )
        } else {
            (
                Some((settings.controls.crouched_idle.as_str(), false)),
                false,
                false,
            )
        }
    } else if was_climbing && wall_ahead && (jump_held || horizontal_input) {
        (
            Some((settings.controls.climb.as_str(), backwards)),
            false,
            true,
        )
    } else if jump_pressed && wall_ahead {
        (Some((settings.controls.climb.as_str(), false)), false, true)
    } else if !grounded && descending {
        (Some((settings.controls.fall.as_str(), false)), false, false)
    } else if was_climbing && !grounded {
        (Some((settings.controls.fall.as_str(), false)), false, false)
    } else if jump_pressed && grounded {
        (
            Some((settings.controls.jump.as_str(), false)),
            settings.temporary_assists.jump_launch.enabled,
            false,
        )
    } else if shift && run_input && !key_s {
        (Some((settings.controls.run.as_str(), false)), false, false)
    } else if horizontal_input {
        (
            Some((
                settings.controls.walk.as_str(),
                backwards && settings.controls.walk_backwards_reverse,
            )),
            false,
            false,
        )
    } else if !grounded && action_state.action_name == settings.controls.jump {
        (Some((settings.controls.jump.as_str(), false)), false, false)
    } else {
        (Some((settings.controls.idle.as_str(), false)), false, false)
    };

    if let Some((action, reverse)) = selected {
        let available_action = if motions.action(action).is_some() {
            action
        } else {
            settings.controls.idle.as_str()
        };
        action_state.select(available_action, available_action == action && reverse);
    }

    // Latch one-shot input until the next fixed physics tick consumes the impulse.
    // FixedUpdate may execute before Update in a given render frame.
    if launch_jump {
        action_state.jump_launch_pending = true;
    }
    action_state.wall_grip_active = apply_wall_grip && wall_ahead;
    action_state.wall_grip_normal = wall_hit.map_or(Vec3::ZERO, |hit| hit.normal);
}

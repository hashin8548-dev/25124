use avian3d::prelude::{Dir3, Forces, LinearVelocity, SpatialQuery, SpatialQueryFilter};
use bevy::prelude::*;

use super::{
    HumanActionState, HumanBodyPart, HumanMotionLibrary, HumanPhysicsSettings,
    HumanPlayer, HumanPlayerRoot, HumanRig,
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
    rig: Option<Res<HumanRig>>,
    settings: Option<Res<HumanPhysicsSettings>>,
    player: Option<Res<HumanPlayer>>,
    root: Query<(&Transform, &LinearVelocity, Forces), With<HumanPlayerRoot>>,
    mut limbs: Query<(&HumanBodyPart, Forces), Without<HumanPlayerRoot>>,
) {
    let (Some(mut action_state), Some(motions), Some(rig), Some(settings), Some(player)) =
        (action_state, motions, rig, settings, player)
    else {
        return;
    };
    let Ok((root_transform, root_velocity, mut root_forces)) = root.single() else {
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

    let mut selected: Option<(&str, bool)> = None;
    let mut launch_jump = false;
    let mut apply_wall_grip = false;

    if crouching {
        if horizontal_input {
            selected = Some((
                settings.controls.crouch.as_str(),
                backwards && settings.controls.crouch_backwards_reverse,
            ));
        } else {
            selected = Some((settings.controls.crouched_idle.as_str(), false));
        }
    } else if was_climbing && wall_ahead && (jump_held || horizontal_input) {
        selected = Some((settings.controls.climb.as_str(), backwards));
        apply_wall_grip = true;
    } else if jump_pressed && wall_ahead {
        selected = Some((settings.controls.climb.as_str(), false));
        apply_wall_grip = true;
    } else if !grounded && descending {
        selected = Some((settings.controls.fall.as_str(), false));
    } else if was_climbing && !grounded {
        selected = Some((settings.controls.fall.as_str(), false));
    } else if jump_pressed {
        selected = Some((settings.controls.jump.as_str(), false));
        launch_jump = settings.temporary_assists.jump_launch.enabled;
    } else if shift && run_input && !key_s {
        selected = Some((settings.controls.run.as_str(), false));
    } else if horizontal_input {
        selected = Some((
            settings.controls.walk.as_str(),
            backwards && settings.controls.walk_backwards_reverse,
        ));
    } else if !grounded && action_state.action_name == settings.controls.jump {
        selected = Some((settings.controls.jump.as_str(), false));
    } else {
        selected = Some((settings.controls.idle.as_str(), false));
    }

    if let Some((action, reverse)) = selected {
        action_state.select(action, reverse);
    }

    if launch_jump {
        if let Some(geometry) = rig.bone("Root").and_then(|bone| bone.collision_geometry.as_ref()) {
            let radius = geometry.radius_m;
            let segment = geometry.capsule_segment_length_m;
            let volume = std::f32::consts::PI * radius * radius * segment
                + (4.0 / 3.0) * std::f32::consts::PI * radius.powi(3);
            let estimated_mass = volume * settings.density_kg_m3;
            let impulse = Vec3::Y
                * estimated_mass
                * settings.temporary_assists.jump_launch.launch_speed_m_s;
            root_forces.apply_linear_impulse(impulse);
        }
    }

    if apply_wall_grip {
        if let Some(hit) = wall_hit {
            // Grip is a physical push into the wall. Surface friction and gravity determine
            // whether the current climbing pose can support the body; no vertical teleport.
            let grip_force = -hit.normal * settings.climb_contact_push_force_n * 0.5;
            for (part, mut forces) in &mut limbs {
                if part.bone_name == "Hand_L" || part.bone_name == "Hand_R" {
                    forces.apply_force(grip_force);
                }
            }
        }
    }
}

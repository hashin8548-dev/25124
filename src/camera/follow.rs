use bevy::{input::mouse::AccumulatedMouseMotion, prelude::*};

use crate::{
    engine::AppMode,
    human::{HumanActionState, HumanPhysicsSettings, HumanPlayer, HumanPlayerRoot},
    ui::MenuState,
};

use super::free_fly::{CameraSpring, FreeFlyCamera};

#[derive(Component)]
struct PlayerCrosshair;

pub(super) fn follow_player_camera(
    mode: Res<State<AppMode>>,
    menu: Res<MenuState>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    config: Res<crate::config::RuntimeConfig>,
    settings: Option<Res<HumanPhysicsSettings>>,
    player: Option<Res<HumanPlayer>>,
    mut action_state: Option<ResMut<HumanActionState>>,
    root: Query<&Transform, With<HumanPlayerRoot>>,
    mut cameras: Query<(&mut Transform, &mut Projection, &mut FreeFlyCamera, &mut CameraSpring)>,
    time: Res<Time>,
) {
    if *mode.get() != AppMode::Simulation || menu.visible {
        return;
    }
    let (Some(settings), Some(player), Some(mut action_state)) =
        (settings, player, action_state)
    else {
        return;
    };
    let Ok(root) = root.get(player.root) else {
        return;
    };
    let Ok((mut camera_transform, mut projection, mut controller, mut spring)) =
        cameras.single_mut()
    else {
        return;
    };

    let sensitivity = config.engine.camera.mouse_radians_per_pixel;
    controller.yaw_radians -= mouse_motion.delta.x * sensitivity;
    controller.pitch_radians =
        (controller.pitch_radians - mouse_motion.delta.y * sensitivity).clamp(-0.75, 0.2);
    action_state.heading_yaw_radians = controller.yaw_radians;

    let yaw_rotation = Quat::from_rotation_y(controller.yaw_radians);
    let target = root.translation + Vec3::from_array(settings.camera.look_at_offset_m);
    let shoulder = yaw_rotation * Vec3::from_array(settings.camera.shoulder_offset_m);
    let trailing = yaw_rotation * Vec3::Z * settings.camera.trailing_distance_m;
    let desired_position = target + shoulder + trailing;

    let dt = time.delta_secs().min(0.05);
    let stiffness = settings.camera.spring_stiffness;
    let damping = settings.camera.spring_damping;
    let acceleration =
        (desired_position - camera_transform.translation) * stiffness - spring.velocity * damping;
    spring.velocity += acceleration * dt;
    camera_transform.translation += spring.velocity * dt;

    camera_transform.rotation = Quat::from_euler(
        EulerRot::YXZ,
        controller.yaw_radians,
        controller.pitch_radians,
        0.0,
    );
    if let Projection::Perspective(perspective) = &mut *projection {
        perspective.fov = settings.camera.vertical_fov_degrees.to_radians();
    }
}

pub(super) fn spawn_player_crosshair(
    mut commands: Commands,
    settings: Option<Res<HumanPhysicsSettings>>,
) {
    let diameter = settings
        .as_ref()
        .map_or(3.0, |value| value.camera.crosshair_diameter_px);
    commands
        .spawn((
            PlayerCrosshair,
            GlobalZIndex(100),
            Node {
                position_type: PositionType::Absolute,
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                ..default()
            },
            BackgroundColor(Color::NONE),
        ))
        .with_children(|children| {
            children.spawn((
                Node {
                    width: Val::Px(diameter),
                    height: Val::Px(diameter),
                    border_radius: BorderRadius::all(Val::Px(diameter * 0.5)),
                    ..default()
                },
                BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.92)),
            ));
        });
}

pub(super) fn sync_player_crosshair(
    menu: Res<MenuState>,
    mut crosshair: Query<&mut Visibility, With<PlayerCrosshair>>,
) {
    for mut visibility in &mut crosshair {
        *visibility = if menu.visible {
            Visibility::Hidden
        } else {
            Visibility::Visible
        };
    }
}

pub(super) fn despawn_player_crosshair(
    mut commands: Commands,
    crosshair: Query<Entity, With<PlayerCrosshair>>,
) {
    for entity in &crosshair {
        commands.entity(entity).despawn();
    }
}

mod free_fly;

pub use free_fly::FreeFlyCamera;

use bevy::prelude::*;

#[derive(SystemSet, Clone, Debug, PartialEq, Eq, Hash)]
pub(crate) struct CameraUpdateSet;

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, free_fly::spawn_camera)
            .add_systems(
                Update,
                (
                    free_fly::sync_camera_settings,
                    free_fly::update_cursor_grab,
                    free_fly::move_camera,
                )
                    .chain()
                    .in_set(CameraUpdateSet),
            );
    }
}

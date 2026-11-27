mod camera;
mod config;
mod engine;
mod human;
mod persistence;
mod physics;
mod rendering;
mod ui;
mod world;

use anyhow::Result;
use bevy::{asset::AssetPlugin, math::DVec3, prelude::*};

use crate::{
    config::{BodyType, RuntimeConfig, WorldLibrary},
    engine::{AppMode, EnginePlugin},
    persistence::WorldStateFile,
    world::{WorldOrigin, parse_voxel_instance_id},
};

fn main() {
    if let Err(error) = run() {
        eprintln!("Fatal startup error: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let Some(config) = RuntimeConfig::from_environment()? else {
        return Ok(());
    };
    let start_in_library = config.start_in_library;
    let world_state = if start_in_library {
        // The first valid world only bootstraps shared engine/content settings. Do not read its
        // save file until the user explicitly selects it in the library.
        WorldStateFile::new(config.world.id.clone())
    } else {
        let state = WorldStateFile::load_or_default(&config.paths.world_state, &config.world.id)?;
        report_saved_state_compatibility(&config, &state);
        state
    };
    let world_origin = WorldOrigin(DVec3::from_array(world_state.origin_m));
    let library = WorldLibrary::discover(&config.paths.project_root)?;
    for warning in &library.warnings {
        warn!("World library entry was skipped: {warning}");
    }
    let asset_root = config.paths.assets_dir.to_string_lossy().into_owned();

    let mut app = App::new();
    app.insert_resource(config)
        .insert_resource(library)
        .insert_resource(world_state)
        .insert_resource(world_origin)
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            file_path: asset_root,
            ..default()
        }))
        .add_plugins(EnginePlugin);

    if !start_in_library {
        app.insert_resource(NextState::Pending(AppMode::Simulation));
    }
    app.run();

    Ok(())
}

fn report_saved_state_compatibility(config: &RuntimeConfig, world_state: &WorldStateFile) {
    for id in world_state.bodies.keys() {
        if let Some((chunk_id, coordinate)) = parse_voxel_instance_id(id)
            && config.chunks.iter().any(|chunk| {
                chunk.id == chunk_id
                    && coordinate[0] < chunk.dimensions[0]
                    && coordinate[1] < chunk.dimensions[1]
                    && coordinate[2] < chunk.dimensions[2]
            })
        {
            continue;
        }

        let Some(instance) = config
            .world
            .objects
            .iter()
            .find(|instance| instance.id == *id)
        else {
            eprintln!(
                "Warning: save contains unresolved object id '{id}'. Its state is preserved for content that may be loaded later."
            );
            continue;
        };

        let definition = config
            .components
            .definitions
            .get(&instance.definition)
            .expect("world definitions are validated before app startup");
        if matches!(definition.body_type, BodyType::Static) {
            eprintln!(
                "Warning: ignoring persisted state for static object '{id}'; its transform is authored by the world manifest. The stale record will be removed on the next save."
            );
        }
    }
}

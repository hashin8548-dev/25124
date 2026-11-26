use std::{
    collections::HashSet,
    env, fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, bail, ensure};

use super::{
    schema::{
        ChunkFile, ComponentCatalog, EngineSettings, MaterialCatalog, RegionManifest, WorldManifest,
    },
    validate::validate_all,
};

#[derive(Clone, Debug)]
pub struct ProjectPaths {
    pub root_dir: PathBuf,
    pub data_dir: PathBuf,
    pub assets_dir: PathBuf,
    pub world_manifest: PathBuf,
    pub world_state: PathBuf,
}

#[derive(bevy::prelude::Resource, Clone, Debug)]
pub struct RuntimeConfig {
    pub engine: EngineSettings,
    pub materials: MaterialCatalog,
    pub components: ComponentCatalog,
    pub world: WorldManifest,
    pub chunks: Vec<ChunkFile>,
    pub paths: ProjectPaths,
}

impl RuntimeConfig {
    pub fn from_environment() -> Result<Option<Self>> {
        let mut root_override = None;
        let mut data_override = None;
        let mut assets_override = None;
        let mut world_override = None;

        let mut args = env::args().skip(1);
        while let Some(argument) = args.next() {
            match argument.as_str() {
                "--help" | "-h" => {
                    print_help();
                    return Ok(None);
                }
                "--root" => root_override = Some(next_path(&mut args, "--root")?),
                "--data" => data_override = Some(next_path(&mut args, "--data")?),
                "--assets" => assets_override = Some(next_path(&mut args, "--assets")?),
                "--world" => world_override = Some(next_path(&mut args, "--world")?),
                unknown => bail!("unknown argument '{unknown}'. Run with --help for usage."),
            }
        }

        let root = match root_override {
            Some(path) => path,
            None => detect_project_root()?,
        };
        let data_dir = resolve_path(
            &root,
            data_override.unwrap_or_else(|| PathBuf::from("data")),
        );
        let assets_dir = resolve_path(
            &root,
            assets_override.unwrap_or_else(|| PathBuf::from("assets")),
        );
        let world_manifest = resolve_path(
            &root,
            world_override.unwrap_or_else(|| PathBuf::from("worlds/sandbox/world.json")),
        );

        Self::load(root, data_dir, assets_dir, world_manifest).map(Some)
    }

    pub fn load(
        root_dir: PathBuf,
        data_dir: PathBuf,
        assets_dir: PathBuf,
        world_manifest_path: PathBuf,
    ) -> Result<Self> {
        let engine: EngineSettings = read_json(&data_dir.join("engine.json"))?;
        let materials: MaterialCatalog = read_json(&data_dir.join("materials.json"))?;
        let components: ComponentCatalog = read_json(&data_dir.join("components.json"))?;
        let world: WorldManifest = read_json(&world_manifest_path)?;
        let chunks = load_regions(&world_manifest_path, &world)?;

        validate_all(&engine, &materials, &components, &world, &chunks)
            .context("configuration validation failed")?;

        let world_state_path = world_manifest_path.with_file_name("state.json");

        Ok(Self {
            engine,
            materials,
            components,
            world,
            chunks,
            paths: ProjectPaths {
                root_dir,
                data_dir,
                assets_dir,
                world_manifest: world_manifest_path,
                world_state: world_state_path,
            },
        })
    }
}

fn load_regions(world_path: &Path, world: &WorldManifest) -> Result<Vec<ChunkFile>> {
    let world_directory = world_path
        .parent()
        .context("world manifest path has no parent directory")?;
    let mut region_ids = HashSet::new();
    let mut chunk_ids = HashSet::new();
    let mut chunk_coordinates = HashSet::new();
    let mut chunks = Vec::new();

    for region_reference in &world.regions {
        validate_relative_data_path(region_reference)
            .with_context(|| format!("invalid region path '{region_reference}'"))?;
        let region_path = world_directory.join(region_reference);
        let region: RegionManifest = read_json(&region_path)?;
        ensure!(
            region.schema_version == 1,
            "unsupported schema version {} in region '{}'",
            region.schema_version,
            region.id
        );
        ensure!(!region.id.trim().is_empty(), "region id cannot be empty");
        ensure!(
            region_ids.insert(region.id.clone()),
            "duplicate region id '{}'",
            region.id
        );

        let region_directory = region_path
            .parent()
            .context("region manifest path has no parent directory")?;
        let mut region_chunk_ids = HashSet::new();
        for reference in &region.chunks {
            ensure!(!reference.id.trim().is_empty(), "chunk id cannot be empty");
            ensure!(
                region_chunk_ids.insert(reference.id.as_str()),
                "region '{}' contains duplicate chunk id '{}'",
                region.id,
                reference.id
            );
            validate_relative_data_path(&reference.path).with_context(|| {
                format!(
                    "invalid chunk path '{}' in region '{}'",
                    reference.path, region.id
                )
            })?;

            let chunk_path = region_directory.join(&reference.path);
            let chunk: ChunkFile = read_json(&chunk_path)?;
            ensure!(
                chunk.id == reference.id,
                "region '{}' references chunk '{}' but file '{}' declares id '{}'",
                region.id,
                reference.id,
                chunk_path.display(),
                chunk.id
            );
            ensure!(
                chunk.coordinate == reference.coordinate,
                "chunk '{}' coordinate does not match its region index",
                chunk.id
            );
            ensure!(
                chunk_ids.insert(chunk.id.clone()),
                "duplicate chunk id '{}'",
                chunk.id
            );
            ensure!(
                chunk_coordinates.insert(chunk.coordinate),
                "multiple chunks occupy coordinate {:?}",
                chunk.coordinate
            );
            chunks.push(chunk);
        }
    }

    Ok(chunks)
}

fn validate_relative_data_path(path: &str) -> Result<()> {
    ensure!(!path.trim().is_empty(), "path is empty");
    ensure!(
        !path.starts_with('/') && !path.contains('\\') && !path.contains(':'),
        "paths must be relative, use forward slashes, and contain no URI or drive prefix"
    );
    for component in path.split('/') {
        ensure!(
            !component.is_empty() && component != "." && component != "..",
            "paths cannot contain empty, current-directory, or parent-directory components"
        );
    }
    Ok(())
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T> {
    let contents = fs::read_to_string(path)
        .with_context(|| format!("could not read configuration file '{}'", path.display()))?;

    serde_json::from_str(&contents)
        .with_context(|| format!("invalid JSON configuration in '{}'", path.display()))
}

fn next_path(args: &mut impl Iterator<Item = String>, option: &str) -> Result<PathBuf> {
    args.next()
        .map(PathBuf::from)
        .with_context(|| format!("option {option} requires a path"))
}

fn resolve_path(root: &Path, path: PathBuf) -> PathBuf {
    if path.is_absolute() {
        path
    } else {
        root.join(path)
    }
}

fn detect_project_root() -> Result<PathBuf> {
    if let Some(executable_dir) = env::current_exe()
        .ok()
        .and_then(|path| path.parent().map(Path::to_path_buf))
        .filter(|path| path.join("data").is_dir())
    {
        return Ok(executable_dir);
    }

    env::current_dir().context("could not determine the project root")
}

fn print_help() {
    println!(
        "Physics Sandbox\n\
         Usage: physics_sandbox [--root PATH] [--data PATH] [--assets PATH] [--world PATH]\n\
         Paths may be absolute or relative to --root.\n"
    );
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use crate::config::RuntimeConfig;

    #[test]
    fn checked_in_configuration_loads_external_chunk_files() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let config = RuntimeConfig::load(
            root.to_path_buf(),
            root.join("data"),
            root.join("assets"),
            root.join("worlds/sandbox/world.json"),
        )
        .expect("sample configuration should validate");

        assert_eq!(config.world.id, "sandbox");
        assert_eq!(config.world.objects.len(), 4);
        assert_eq!(config.chunks.len(), 4);
        assert_eq!(
            config.world.objects[1].definition,
            config.world.objects[2].definition
        );
    }

    #[test]
    fn relative_content_paths_reject_parent_traversal() {
        assert!(super::validate_relative_data_path("regions/overworld.json").is_ok());
        assert!(super::validate_relative_data_path("../outside.json").is_err());
        assert!(super::validate_relative_data_path("C:/outside.json").is_err());
        assert!(super::validate_relative_data_path("regions\\overworld.json").is_err());
    }
}

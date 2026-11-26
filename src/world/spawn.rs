use std::collections::HashMap;

use avian3d::prelude::{
    AngularVelocity, CcdFilter, Collider, ColliderDensity, CollisionEventsEnabled, Friction,
    LinearVelocity, Restitution, RigidBody, SleepThreshold, SweepMode, SweptCcd,
};
use bevy::{gltf::prelude::GltfAssetLabel, prelude::*};

use crate::{
    config::{
        AlphaMode, BodyType, CcdFilterDefinition, ColliderShape, ComponentDefinition,
        RuntimeConfig, SweepModeDefinition, VisualDefinition,
    },
    persistence::{
        PersistedFragment, PersistedFragmentState, PersistRuntimeState, PersistentObjectId,
        WorldStateFile,
    },
    physics::DestructibleBody,
    world::voxel_instance_id,
};

pub(super) fn spawn_world(
    mut commands: Commands,
    config: Res<RuntimeConfig>,
    world_state: Res<WorldStateFile>,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut mesh_cache = HashMap::<MeshKey, Handle<Mesh>>::new();
    let mut material_cache = HashMap::<VisualMaterialKey, Handle<StandardMaterial>>::new();

    for instance in &config.world.objects {
        if world_state.fractured.contains(&instance.id) {
            continue;
        }

        let definition = config
            .components
            .definitions
            .get(&instance.definition)
            .expect("component references are validated before app startup");
        let physical_material = config
            .materials
            .materials
            .get(&definition.material)
            .expect("material references are validated before app startup");

        let saved = if matches!(definition.body_type, BodyType::Dynamic) {
            world_state.bodies.get(&instance.id)
        } else {
            None
        };
        let transform = Transform {
            translation: Vec3::from_array(instance.position_m),
            rotation: rotation_from_degrees(instance.rotation_degrees_xyz),
            ..default()
        };

        spawn_body(
            &mut commands,
            &instance.id,
            &instance.definition,
            definition,
            physical_material,
            transform,
            saved,
            &config,
            &asset_server,
            &mut meshes,
            &mut materials,
            &mut mesh_cache,
            &mut material_cache,
        );
    }

    let mut voxel_count = 0_usize;
    for chunk in &config.chunks {
        let dimensions = chunk.dimensions;
        let voxel_size = chunk.voxel_size_m;
        let chunk_origin = Vec3::new(
            chunk.coordinate[0] as f32 * dimensions[0] as f32 * voxel_size,
            chunk.coordinate[1] as f32 * dimensions[1] as f32 * voxel_size,
            chunk.coordinate[2] as f32 * dimensions[2] as f32 * voxel_size,
        );

        for layer in &chunk.layers {
            for (z, row) in layer.rows.iter().enumerate() {
                for (x, character) in row.chars().enumerate() {
                    let palette_index = character
                        .to_digit(16)
                        .expect("chunk palette rows are validated before app startup")
                        as usize;
                    if palette_index == 0 {
                        continue;
                    }

                    let definition_id = &chunk.palette[palette_index];
                    let instance_id = voxel_instance_id(
                        &chunk.id,
                        x as u32,
                        layer.y,
                        z as u32,
                    );
                    if world_state.fractured.contains(&instance_id) {
                        continue;
                    }

                    let authored_definition = config
                        .components
                        .definitions
                        .get(definition_id)
                        .expect("chunk palettes are validated before app startup");
                    let physical_material = config
                        .materials
                        .materials
                        .get(&authored_definition.material)
                        .expect("component materials are validated before app startup");

                    // Voxel dimensions are scaled from the palette definition by this chunk's
                    // authored voxel size. The chunk file remains independent of the block catalogue.
                    let mut definition = authored_definition.clone();
                    let ColliderShape::Cuboid { half_extents } = &mut definition.collider else {
                        unreachable!("chunk palette definitions are validated as cuboids");
                    };
                    for extent in half_extents {
                        *extent *= voxel_size;
                    }

                    let saved = if matches!(definition.body_type, BodyType::Dynamic) {
                        world_state.bodies.get(&instance_id)
                    } else {
                        None
                    };
                    let position = chunk_origin
                        + Vec3::new(
                            (x as f32 + 0.5) * voxel_size,
                            (layer.y as f32 + 0.5) * voxel_size,
                            (z as f32 + 0.5) * voxel_size,
                        );

                    spawn_body(
                        &mut commands,
                        &instance_id,
                        definition_id,
                        &definition,
                        physical_material,
                        Transform::from_translation(position),
                        saved,
                        &config,
                        &asset_server,
                        &mut meshes,
                        &mut materials,
                        &mut mesh_cache,
                        &mut material_cache,
                    );
                    voxel_count += 1;
                }
            }
        }
    }

    let mut restored_fragment_count = 0_usize;
    for (instance_id, state) in &world_state.fragments {
        let Some(definition) = config.components.definitions.get(&state.definition_id) else {
            warn!(
                "Skipping fragment '{}' because component definition '{}' is unavailable",
                instance_id,
                state.definition_id
            );
            continue;
        };
        let Some(physical_material) = config.materials.materials.get(&definition.material) else {
            warn!(
                "Skipping fragment '{}' because material '{}' is unavailable",
                instance_id,
                definition.material
            );
            continue;
        };

        spawn_saved_fragment(
            &mut commands,
            instance_id,
            state,
            definition,
            physical_material,
            &config,
            &asset_server,
            &mut meshes,
            &mut materials,
            &mut mesh_cache,
            &mut material_cache,
        );
        restored_fragment_count += 1;
    }

    info!(
        "Loaded world '{}' with {} authored objects, {} voxel cells across {} chunks, and {} saved fragments",
        config.world.name,
        config.world.objects.len(),
        voxel_count,
        config.chunks.len(),
        restored_fragment_count
    );
}

#[allow(clippy::too_many_arguments)]
fn spawn_body(
    commands: &mut Commands,
    instance_id: &str,
    definition_id: &str,
    definition: &ComponentDefinition,
    physical_material: &crate::config::MaterialDefinition,
    authored_transform: Transform,
    saved: Option<&crate::persistence::state::PersistedBodyState>,
    config: &RuntimeConfig,
    asset_server: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_cache: &mut HashMap<MeshKey, Handle<Mesh>>,
    material_cache: &mut HashMap<VisualMaterialKey, Handle<StandardMaterial>>,
) {
    let (translation, rotation, linear_velocity, angular_velocity) = saved
        .map(|state| {
            (
                Vec3::from_array(state.translation_m),
                Quat::from_array(state.rotation_xyzw),
                Vec3::from_array(state.linear_velocity_m_s),
                Vec3::from_array(state.angular_velocity_rad_s),
            )
        })
        .unwrap_or((
            authored_transform.translation,
            authored_transform.rotation,
            Vec3::ZERO,
            Vec3::ZERO,
        ));

    let rigid_body = match definition.body_type {
        BodyType::Static => RigidBody::Static,
        BodyType::Dynamic => RigidBody::Dynamic,
    };
    let collider = collider_from_definition(&definition.collider);
    let friction = Friction::new(physical_material.dynamic_friction)
        .with_static_coefficient(physical_material.static_friction);

    let mut entity = commands.spawn((
        Name::new(instance_id.to_owned()),
        PersistentObjectId(instance_id.to_owned()),
        rigid_body,
        collider,
        ColliderDensity(physical_material.density_kg_m3),
        friction,
        Restitution::new(physical_material.restitution),
        CollisionEventsEnabled,
        LinearVelocity(linear_velocity),
        AngularVelocity(angular_velocity),
        Transform {
            translation,
            rotation,
            ..default()
        },
    ));

    if matches!(definition.body_type, BodyType::Dynamic) {
        entity.insert((
            PersistRuntimeState,
            SleepThreshold {
                linear: config.engine.physics.sleep_linear_threshold_m_s,
                angular: config.engine.physics.sleep_angular_threshold_rad_s,
            },
        ));
    }

    if let Some(ccd) = &definition.swept_ccd {
        let filter = match ccd.filter {
            CcdFilterDefinition::Default => CcdFilter::DEFAULT,
            CcdFilterDefinition::Dynamic => CcdFilter::DYNAMIC,
            CcdFilterDefinition::All => CcdFilter::ALL,
            CcdFilterDefinition::None => CcdFilter::NONE,
        };
        let mode = match ccd.mode {
            SweepModeDefinition::Linear => SweepMode::Linear,
            SweepModeDefinition::NonLinear => SweepMode::NonLinear,
        };
        entity.insert(SweptCcd {
            filter,
            mode,
            threshold: ccd.threshold,
        });
    }

    if let (
        Some(destructibility),
        ColliderShape::Cuboid { half_extents },
    ) = (&definition.destructibility, &definition.collider)
    {
        entity.insert(DestructibleBody::new(
            instance_id,
            definition_id,
            destructibility,
            Vec3::from_array(*half_extents),
            definition.body_type,
            physical_material.density_kg_m3,
            physical_material.static_friction,
            physical_material.dynamic_friction,
            physical_material.restitution,
            &definition.visual,
        ));
    }

    add_visual(
        &mut entity,
        definition,
        asset_server,
        meshes,
        materials,
        mesh_cache,
        material_cache,
    );
}

#[allow(clippy::too_many_arguments)]
fn spawn_saved_fragment(
    commands: &mut Commands,
    instance_id: &str,
    state: &PersistedFragmentState,
    authored_definition: &ComponentDefinition,
    physical_material: &crate::config::MaterialDefinition,
    config: &RuntimeConfig,
    asset_server: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_cache: &mut HashMap<MeshKey, Handle<Mesh>>,
    material_cache: &mut HashMap<VisualMaterialKey, Handle<StandardMaterial>>,
) {
    let half_extents = Vec3::from_array(state.half_extents_m);
    let mut definition = authored_definition.clone();
    definition.collider = ColliderShape::Cuboid {
        half_extents: state.half_extents_m,
    };
    definition.destructibility = None;

    let mut entity = commands.spawn((
        Name::new(instance_id.to_owned()),
        PersistentObjectId(instance_id.to_owned()),
        PersistedFragment {
            definition_id: state.definition_id.clone(),
            half_extents_m: state.half_extents_m,
        },
        RigidBody::Dynamic,
        Collider::cuboid(
            half_extents.x * 2.0,
            half_extents.y * 2.0,
            half_extents.z * 2.0,
        ),
        ColliderDensity(physical_material.density_kg_m3),
        Friction::new(physical_material.dynamic_friction)
            .with_static_coefficient(physical_material.static_friction),
        Restitution::new(physical_material.restitution),
        CollisionEventsEnabled,
        LinearVelocity(Vec3::from_array(state.linear_velocity_m_s)),
        AngularVelocity(Vec3::from_array(state.angular_velocity_rad_s)),
        SleepThreshold {
            linear: config.engine.physics.sleep_linear_threshold_m_s,
            angular: config.engine.physics.sleep_angular_threshold_rad_s,
        },
        Transform {
            translation: Vec3::from_array(state.translation_m),
            rotation: Quat::from_array(state.rotation_xyzw),
            ..default()
        },
    ));

    add_visual(
        &mut entity,
        &definition,
        asset_server,
        meshes,
        materials,
        mesh_cache,
        material_cache,
    );
}

fn add_visual(
    entity: &mut EntityCommands,
    definition: &ComponentDefinition,
    asset_server: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_cache: &mut HashMap<MeshKey, Handle<Mesh>>,
    material_cache: &mut HashMap<VisualMaterialKey, Handle<StandardMaterial>>,
) {
    if let Some(model_path) = &definition.visual.model {
        let (asset_path, scene_index) = match model_path.split_once('#') {
            Some((path, selector)) => {
                let index = selector
                    .strip_prefix("Scene")
                    .and_then(|value| value.parse::<usize>().ok())
                    .expect("model scene selectors are validated before app startup");
                (path, index)
            }
            None => (model_path.as_str(), 0),
        };
        let scene_path = GltfAssetLabel::Scene(scene_index).from_asset(asset_path.to_owned());
        entity.with_children(|children| {
            children.spawn((
                WorldAssetRoot(asset_server.load(scene_path)),
                Transform::default(),
            ));
        });
        return;
    }

    let mesh_key = MeshKey::from_shape(&definition.collider);
    let mesh = mesh_cache
        .entry(mesh_key)
        .or_insert_with(|| mesh_from_collider_shape(&definition.collider, meshes))
        .clone();

    let material_key = VisualMaterialKey::from_visual(&definition.visual);
    let material = material_cache
        .entry(material_key)
        .or_insert_with(|| materials.add(standard_material_from_visual(&definition.visual)))
        .clone();

    entity.insert((Mesh3d(mesh), MeshMaterial3d(material)));
}

fn standard_material_from_visual(visual: &VisualDefinition) -> StandardMaterial {
    StandardMaterial {
        base_color: Color::srgba(
            visual.color_srgba[0],
            visual.color_srgba[1],
            visual.color_srgba[2],
            visual.color_srgba[3],
        ),
        perceptual_roughness: visual.perceptual_roughness,
        metallic: visual.metallic,
        alpha_mode: match visual.alpha_mode {
            AlphaMode::Opaque => bevy::prelude::AlphaMode::Opaque,
            AlphaMode::Blend => bevy::prelude::AlphaMode::Blend,
        },
        ..default()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum MeshKey {
    Cuboid([u32; 3]),
    Sphere(u32),
    Capsule(u32, u32),
}

impl MeshKey {
    fn from_shape(shape: &ColliderShape) -> Self {
        match shape {
            ColliderShape::Cuboid { half_extents } => Self::Cuboid([
                half_extents[0].to_bits(),
                half_extents[1].to_bits(),
                half_extents[2].to_bits(),
            ]),
            ColliderShape::Sphere { radius } => Self::Sphere(radius.to_bits()),
            ColliderShape::Capsule {
                radius,
                segment_length,
            } => Self::Capsule(radius.to_bits(), segment_length.to_bits()),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct VisualMaterialKey {
    color: [u32; 4],
    roughness: u32,
    metallic: u32,
    alpha_mode: AlphaMode,
}

impl VisualMaterialKey {
    fn from_visual(visual: &VisualDefinition) -> Self {
        Self {
            color: visual.color_srgba.map(f32::to_bits),
            roughness: visual.perceptual_roughness.to_bits(),
            metallic: visual.metallic.to_bits(),
            alpha_mode: visual.alpha_mode,
        }
    }
}

fn collider_from_definition(shape: &ColliderShape) -> Collider {
    match shape {
        ColliderShape::Cuboid { half_extents } => Collider::cuboid(
            half_extents[0] * 2.0,
            half_extents[1] * 2.0,
            half_extents[2] * 2.0,
        ),
        ColliderShape::Sphere { radius } => Collider::sphere(*radius),
        ColliderShape::Capsule {
            radius,
            segment_length,
        } => Collider::capsule(*radius, *segment_length),
    }
}

fn mesh_from_collider_shape(shape: &ColliderShape, meshes: &mut Assets<Mesh>) -> Handle<Mesh> {
    match shape {
        ColliderShape::Cuboid { half_extents } => meshes.add(Cuboid::new(
            half_extents[0] * 2.0,
            half_extents[1] * 2.0,
            half_extents[2] * 2.0,
        )),
        ColliderShape::Sphere { radius } => meshes.add(Sphere::new(*radius).mesh().uv(48, 32)),
        ColliderShape::Capsule {
            radius,
            segment_length,
        } => meshes.add(Capsule3d::new(*radius, *segment_length)),
    }
}

fn rotation_from_degrees(rotation_xyz: [f32; 3]) -> Quat {
    Quat::from_euler(
        EulerRot::XYZ,
        rotation_xyz[0].to_radians(),
        rotation_xyz[1].to_radians(),
        rotation_xyz[2].to_radians(),
    )
}

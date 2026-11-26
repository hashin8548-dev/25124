use std::collections::BTreeSet;

use avian3d::prelude::{
    AngularVelocity, Collider, ColliderDensity, CollisionEventsEnabled, Collisions, Friction,
    LinearVelocity, Restitution, RigidBody, SleepThreshold,
};
use bevy::prelude::*;

use crate::{
    config::{AlphaMode, BodyType, DestructibilityDefinition, VisualDefinition},
    persistence::{PersistedFragment, PersistentObjectId, WorldStateFile},
    physics::VoxelBond,
};

#[derive(Component, Clone, Debug)]
pub struct DestructibleBody {
    pub instance_id: String,
    pub definition_id: String,
    pub impact_speed_threshold_m_s: f32,
    pub fragments_per_axis: [u32; 3],
    pub half_extents_m: Vec3,
    pub body_type: BodyType,
    pub density_kg_m3: f32,
    pub static_friction: f32,
    pub dynamic_friction: f32,
    pub restitution: f32,
    pub visual: VisualDefinition,
}

impl DestructibleBody {
    pub fn new(
        instance_id: impl Into<String>,
        definition_id: impl Into<String>,
        definition: &DestructibilityDefinition,
        half_extents_m: Vec3,
        body_type: BodyType,
        density_kg_m3: f32,
        static_friction: f32,
        dynamic_friction: f32,
        restitution: f32,
        visual: &VisualDefinition,
    ) -> Self {
        Self {
            instance_id: instance_id.into(),
            definition_id: definition_id.into(),
            impact_speed_threshold_m_s: definition.impact_speed_threshold_m_s,
            fragments_per_axis: definition.fragments_per_axis,
            half_extents_m,
            body_type,
            density_kg_m3,
            static_friction,
            dynamic_friction,
            restitution,
            visual: visual.clone(),
        }
    }
}

#[derive(Resource, Clone, Debug, Default)]
pub struct DestructionJournal {
    pub fractured: BTreeSet<String>,
    pub broken_bonds: BTreeSet<String>,
}

#[derive(Clone, Copy, Debug, Default)]
struct ImpactTransfer {
    speed_m_s: f32,
    linear_impulse_ns: Vec3,
    angular_impulse_nms: Vec3,
}

pub(super) fn restore_destruction_journal(
    state: Res<WorldStateFile>,
    mut journal: ResMut<DestructionJournal>,
) {
    journal.fractured.clone_from(&state.fractured);
    journal.broken_bonds.clone_from(&state.broken_bonds);
}

pub(super) fn fracture_on_new_impacts(
    mut commands: Commands,
    collisions: Collisions,
    mut journal: ResMut<DestructionJournal>,
    targets: Query<(&DestructibleBody, &Transform, &LinearVelocity, &AngularVelocity)>,
    config: Res<crate::config::RuntimeConfig>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    bonds: Query<(Entity, &VoxelBond)>,
) {
    let mut candidates = std::collections::HashMap::<Entity, ImpactTransfer>::new();

    for pair in collisions.iter().filter(|pair| pair.collision_started()) {
        let mut impact_speed = 0.0_f32;
        let mut impulse_on_first = Vec3::ZERO;
        let mut torque_on_first = Vec3::ZERO;

        for manifold in &pair.manifolds {
            for point in &manifold.points {
                impact_speed = impact_speed.max((-point.normal_speed).max(0.0));
                // The manifold normal points from collider1 to collider2. The normal
                // impulse applied to collider1 therefore acts opposite that normal.
                let impulse = -manifold.normal * point.normal_impulse;
                impulse_on_first += impulse;
                torque_on_first += point.point.cross(impulse);
            }
        }

        if impact_speed <= 0.0 {
            continue;
        }

        for (target_entity, sign) in [(pair.collider1, 1.0), (pair.collider2, -1.0)] {
            let Ok((target, transform, _, _)) = targets.get(target_entity) else {
                continue;
            };
            if impact_speed < target.impact_speed_threshold_m_s
                || journal.fractured.contains(&target.instance_id)
            {
                continue;
            }

            // Avian's contact normal impulse is the impulse on collider1. The equal and
            // opposite impulse is used when collider2 is the destructible body.
            let transfer = ImpactTransfer {
                speed_m_s: impact_speed,
                linear_impulse_ns: impulse_on_first * sign,
                angular_impulse_nms: (torque_on_first
                    - transform.translation.cross(impulse_on_first))
                    * sign,
            };
            candidates
                .entry(target_entity)
                .and_modify(|existing| {
                    if transfer.speed_m_s > existing.speed_m_s {
                        existing.speed_m_s = transfer.speed_m_s;
                    }
                    existing.linear_impulse_ns += transfer.linear_impulse_ns;
                    existing.angular_impulse_nms += transfer.angular_impulse_nms;
                })
                .or_insert(transfer);
        }
    }

    for (entity, impact) in candidates {
        let Ok((target, transform, linear_velocity, angular_velocity)) = targets.get(entity) else {
            continue;
        };
        let target = target.clone();
        if journal.fractured.contains(&target.instance_id) {
            continue;
        }

        let half = target.half_extents_m;
        let full_size = half * 2.0;
        let total_mass = full_size.x * full_size.y * full_size.z * target.density_kg_m3;
        if !total_mass.is_finite() || total_mass <= 0.0 {
            warn!("Skipping fracture for '{}' because its mass is invalid", target.instance_id);
            continue;
        }

        let mut inherited_linear_velocity = linear_velocity.0;
        let mut inherited_angular_velocity = angular_velocity.0;

        // A static body has no dynamic velocity in Avian. When it fractures, transfer the
        // measured contact impulse to the newly-created fragment mass. Dynamic bodies already
        // contain the solver's post-impact velocity, so their impulse must not be applied twice.
        if matches!(target.body_type, BodyType::Static) {
            inherited_linear_velocity += impact.linear_impulse_ns / total_mass;

            let inertia_local = Vec3::new(
                total_mass * (half.y * half.y + half.z * half.z) / 3.0,
                total_mass * (half.x * half.x + half.z * half.z) / 3.0,
                total_mass * (half.x * half.x + half.y * half.y) / 3.0,
            );
            let local_angular_impulse = transform.rotation.inverse() * impact.angular_impulse_nms;
            let angular_delta_local = Vec3::new(
                local_angular_impulse.x / inertia_local.x,
                local_angular_impulse.y / inertia_local.y,
                local_angular_impulse.z / inertia_local.z,
            );
            inherited_angular_velocity += transform.rotation * angular_delta_local;
        }

        let counts = target.fragments_per_axis;
        let fragment_half_extents = Vec3::new(
            full_size.x / counts[0] as f32 * 0.5,
            full_size.y / counts[1] as f32 * 0.5,
            full_size.z / counts[2] as f32 * 0.5,
        );
        let fragment_mesh = meshes.add(Cuboid::new(
            fragment_half_extents.x * 2.0,
            fragment_half_extents.y * 2.0,
            fragment_half_extents.z * 2.0,
        ));
        let fragment_material = materials.add(standard_material_from_visual(&target.visual));

        let mut fragment_index = 0_u32;
        for x in 0..counts[0] {
            for y in 0..counts[1] {
                for z in 0..counts[2] {
                    let local_offset = Vec3::new(
                        ((x as f32 + 0.5) / counts[0] as f32 * 2.0 - 1.0) * half.x,
                        ((y as f32 + 0.5) / counts[1] as f32 * 2.0 - 1.0) * half.y,
                        ((z as f32 + 0.5) / counts[2] as f32 * 2.0 - 1.0) * half.z,
                    );
                    let world_offset = transform.rotation * local_offset;
                    let fragment_id =
                        format!("fragment:{}:{fragment_index}", target.instance_id);
                    let rotation = transform.rotation;
                    let fragment_linear_velocity =
                        inherited_linear_velocity + inherited_angular_velocity.cross(world_offset);

                    commands.spawn((
                        Name::new(fragment_id.clone()),
                        PersistentObjectId(fragment_id),
                        PersistedFragment {
                            definition_id: target.definition_id.clone(),
                            half_extents_m: fragment_half_extents.to_array(),
                        },
                        RigidBody::Dynamic,
                        Collider::cuboid(
                            fragment_half_extents.x * 2.0,
                            fragment_half_extents.y * 2.0,
                            fragment_half_extents.z * 2.0,
                        ),
                        ColliderDensity(target.density_kg_m3),
                        Friction::new(target.dynamic_friction)
                            .with_static_coefficient(target.static_friction),
                        Restitution::new(target.restitution),
                        CollisionEventsEnabled,
                        LinearVelocity(fragment_linear_velocity),
                        AngularVelocity(inherited_angular_velocity),
                        SleepThreshold {
                            linear: config.engine.physics.sleep_linear_threshold_m_s,
                            angular: config.engine.physics.sleep_angular_threshold_rad_s,
                        },
                        Mesh3d(fragment_mesh.clone()),
                        MeshMaterial3d(fragment_material.clone()),
                        Transform {
                            translation: transform.translation + world_offset,
                            rotation,
                            ..default()
                        },
                    ));
                    fragment_index += 1;
                }
            }
        }

        journal.fractured.insert(target.instance_id.clone());
        for (joint_entity, bond) in &bonds {
            if bond.body1 == entity || bond.body2 == entity {
                journal.broken_bonds.insert(bond.id.clone());
                commands.entity(joint_entity).despawn();
            }
        }
        commands.entity(entity).despawn();
        info!(
            "Fractured '{}' into {} pieces after a {:.2} m/s impact",
            target.instance_id,
            fragment_index,
            impact.speed_m_s
        );
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fragment_grid_preserves_the_original_collider_volume() {
        let original = Vec3::new(0.35, 0.35, 0.35);
        for counts in [[2, 2, 2], [2, 3, 4], [4, 4, 4]] {
            let fragment_half = Vec3::new(
                original.x * 2.0 / counts[0] as f32 * 0.5,
                original.y * 2.0 / counts[1] as f32 * 0.5,
                original.z * 2.0 / counts[2] as f32 * 0.5,
            );
            let fragment_count = (counts[0] * counts[1] * counts[2]) as f32;
            let original_full_size = original * 2.0;
            let fragment_full_size = fragment_half * 2.0;
            let original_volume =
                original_full_size.x * original_full_size.y * original_full_size.z;
            let fragment_volume = fragment_full_size.x
                * fragment_full_size.y
                * fragment_full_size.z
                * fragment_count;
            assert!((original_volume - fragment_volume).abs() < 1.0e-6);
        }
    }
}

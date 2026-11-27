use std::collections::HashMap;

use avian3d::prelude::{
    Collisions, ComputedMass, FixedJoint, Gravity, JointCollisionDisabled, JointForces, RigidBody,
};
use bevy::prelude::*;

use crate::{
    config::BondStrengthDefinition,
    physics::structural::{StructuralEdge, StructuralNode, solve_structural_loads},
};

#[derive(Component, Clone, Debug)]
pub struct VoxelBond {
    pub id: String,
    pub body1: Entity,
    pub body2: Entity,
    pub normal_local: Vec3,
    pub face_size_m: f32,
    pub strength: BondStrengthDefinition,
    /// Accumulated progressive damage in [0, 1].
    pub damage: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BondStress {
    pub tensile_pa: f32,
    pub compressive_pa: f32,
    pub shear_pa: f32,
    pub bending_pa: f32,
    pub torsional_pa: f32,
}

impl BondStress {
    pub fn exceeds(self, strength: &BondStrengthDefinition) -> bool {
        self.tensile_pa > strength.tensile_strength_pa
            || self.compressive_pa > strength.compressive_strength_pa
            || self.shear_pa > strength.shear_strength_pa
            || self.bending_pa > strength.bending_strength_pa
            || self.torsional_pa > strength.torsional_strength_pa
    }

    fn utilization(self, strength: &BondStrengthDefinition) -> f32 {
        (self.tensile_pa / strength.tensile_strength_pa)
            .max(self.compressive_pa / strength.compressive_strength_pa)
            .max(self.shear_pa / strength.shear_strength_pa)
            .max(self.bending_pa / strength.bending_strength_pa)
            .max(self.torsional_pa / strength.torsional_strength_pa)
    }

    fn include_axial_network_stress(&mut self, axial: crate::physics::structural::AxialBondStress) {
        self.tensile_pa = self.tensile_pa.max(axial.tensile_pa);
        self.compressive_pa = self.compressive_pa.max(axial.compressive_pa);
    }
}

/// Converts Avian's joint resultant into nominal section stresses.
///
/// This is a beam/section approximation for a square voxel bond, not a finite-element stress
/// solution. The separate failure modes are kept explicit so the data can tune brittle materials.
pub fn calculate_bond_stress(
    force_world: Vec3,
    torque_world: Vec3,
    normal_world: Vec3,
    face_size_m: f32,
) -> BondStress {
    if !force_world.is_finite()
        || !torque_world.is_finite()
        || !normal_world.is_finite()
        || !face_size_m.is_finite()
        || face_size_m <= 0.0
    {
        return BondStress {
            tensile_pa: f32::INFINITY,
            compressive_pa: f32::INFINITY,
            shear_pa: f32::INFINITY,
            bending_pa: f32::INFINITY,
            torsional_pa: f32::INFINITY,
        };
    }

    let normal = normal_world.normalize_or_zero();
    if normal == Vec3::ZERO {
        return BondStress::default();
    }

    let axial_force = force_world.dot(normal);
    let shear_force = force_world - normal * axial_force;
    let axial_moment = torque_world.dot(normal);
    let bending_moment = torque_world - normal * axial_moment;
    let area = face_size_m * face_size_m;
    let cube = area * face_size_m;

    BondStress {
        tensile_pa: axial_force.max(0.0) / area,
        compressive_pa: (-axial_force).max(0.0) / area,
        shear_pa: shear_force.length() / area,
        bending_pa: 6.0 * bending_moment.length() / cube,
        torsional_pa: 3.0 * axial_moment.abs() / cube,
    }
}

/// Data required to create one rigid structural connection.
#[derive(Clone, Debug)]
pub struct VoxelBondSpec {
    pub id: String,
    pub body1: Entity,
    pub body2: Entity,
    pub local_anchor1: Vec3,
    pub local_anchor2: Vec3,
    pub normal_local: Vec3,
    pub face_size_m: f32,
    pub strength: BondStrengthDefinition,
}

pub fn spawn_voxel_bond(commands: &mut Commands, spec: VoxelBondSpec) -> Entity {
    commands
        .spawn((
            Name::new(spec.id.clone()),
            FixedJoint::new(spec.body1, spec.body2)
                .with_local_anchor1(spec.local_anchor1)
                .with_local_anchor2(spec.local_anchor2),
            JointForces::new(),
            JointCollisionDisabled,
            VoxelBond {
                id: spec.id,
                body1: spec.body1,
                body2: spec.body2,
                normal_local: spec.normal_local,
                face_size_m: spec.face_size_m,
                strength: spec.strength,
                damage: 0.0,
            },
        ))
        .id()
}

// Each query/resource declares a distinct input to the structural solve and damage update.
#[allow(clippy::too_many_arguments)]
pub(super) fn break_overstressed_bonds(
    mut commands: Commands,
    mut bonds: Query<(Entity, &mut VoxelBond, &JointForces)>,
    bodies: Query<(&Transform, &RigidBody, Option<&ComputedMass>)>,
    collisions: Collisions,
    fixed_time: Res<Time<Fixed>>,
    gravity: Res<Gravity>,
    time: Res<Time>,
    mut journal: ResMut<crate::physics::DestructionJournal>,
) {
    let delta_seconds = time.delta_secs().max(0.0);
    if delta_seconds <= 0.0 {
        return;
    }

    let mut snapshot: Vec<_> = bonds
        .iter_mut()
        .map(|(entity, bond, forces)| (entity, bond.clone(), *forces))
        .collect();
    snapshot.sort_by(|left, right| left.1.id.cmp(&right.1.id));

    // Contact normals point from collider 1 to collider 2. The solver applies -n to collider 1
    // and +n to collider 2; convert the measured impulse to a fixed-step average force.
    let fixed_delta = fixed_time.delta_secs().max(1.0e-6);
    let mut contact_forces = HashMap::<Entity, Vec3>::new();
    for pair in collisions.iter() {
        let mut impulse = Vec3::ZERO;
        for manifold in &pair.manifolds {
            let magnitude = manifold
                .points
                .iter()
                .map(|point| point.normal_impulse.max(0.0))
                .sum::<f32>();
            impulse += manifold.normal * magnitude;
        }
        if impulse.is_finite() && impulse.length_squared() > 0.0 {
            *contact_forces.entry(pair.collider1).or_default() -= impulse / fixed_delta;
            *contact_forces.entry(pair.collider2).or_default() += impulse / fixed_delta;
        }
    }

    let mut node_indices = HashMap::<Entity, usize>::new();
    let mut nodes = Vec::<StructuralNode>::new();
    let mut edges = Vec::<StructuralEdge>::new();

    for (_, bond, _) in &snapshot {
        let mut endpoint_indices = [None, None];
        let mut endpoint_entities = [bond.body1, bond.body2];
        endpoint_entities.sort_by_key(|entity| entity.to_bits());
        for (slot, body_entity) in endpoint_entities.into_iter().enumerate() {
            if let Some(index) = node_indices.get(&body_entity).copied() {
                endpoint_indices[slot] = Some(index);
                continue;
            }
            let Ok((transform, rigid_body, mass)) = bodies.get(body_entity) else {
                continue;
            };
            let index = nodes.len();
            node_indices.insert(body_entity, index);
            nodes.push(StructuralNode {
                position_m: transform.translation,
                mass_kg: mass.map_or(0.0, |mass| mass.value()),
                applied_force_n: contact_forces
                    .get(&body_entity)
                    .copied()
                    .unwrap_or(Vec3::ZERO),
                fixed: !matches!(*rigid_body, RigidBody::Dynamic),
            });
            endpoint_indices[slot] = Some(index);
        }

        let [Some(node_a), Some(node_b)] = endpoint_indices else {
            continue;
        };
        edges.push(StructuralEdge {
            id: bond.id.clone(),
            node_a,
            node_b,
            elastic_modulus_pa: bond.strength.elastic_modulus_pa,
            area_m2: bond.face_size_m * bond.face_size_m,
        });
    }

    let network_stress = solve_structural_loads(&nodes, &edges, gravity.0);

    for (joint_entity, mut bond, joint_forces) in &mut bonds {
        if journal.broken_bonds.contains(&bond.id) {
            journal.damaged_bonds.remove(&bond.id);
            commands.entity(joint_entity).despawn();
            continue;
        }

        let Ok((body_transform, _, _)) = bodies.get(bond.body1) else {
            journal.damaged_bonds.remove(&bond.id);
            commands.entity(joint_entity).despawn();
            journal.broken_bonds.insert(bond.id.clone());
            continue;
        };
        if bodies.get(bond.body2).is_err() {
            journal.damaged_bonds.remove(&bond.id);
            commands.entity(joint_entity).despawn();
            journal.broken_bonds.insert(bond.id.clone());
            continue;
        }

        bond.damage = journal
            .damaged_bonds
            .get(&bond.id)
            .copied()
            .unwrap_or(bond.damage)
            .clamp(0.0, 1.0);

        let normal_world = body_transform.rotation * bond.normal_local;
        let mut stress = calculate_bond_stress(
            joint_forces.force(),
            joint_forces.torque(),
            normal_world,
            bond.face_size_m,
        );
        if let Some(axial) = network_stress.get(&bond.id) {
            stress.include_axial_network_stress(*axial);
        }

        let utilization = stress.utilization(&bond.strength);
        if utilization > 0.5 {
            bond.damage = (bond.damage
                + (utilization - 0.5) * bond.strength.damage_rate_per_s * delta_seconds)
                .min(1.0);
        }
        if bond.damage > 0.0 {
            journal.damaged_bonds.insert(bond.id.clone(), bond.damage);
        }

        if stress.exceeds(&bond.strength) || bond.damage >= 1.0 {
            journal.broken_bonds.insert(bond.id.clone());
            journal.damaged_bonds.remove(&bond.id);
            commands.entity(joint_entity).despawn();
            info!(
                "Voxel bond '{}' failed at {:.0}% utilization (damage {:.3}): tension {:.2} MPa, compression {:.2} MPa, shear {:.2} MPa, bending {:.2} MPa, torsion {:.2} MPa",
                bond.id,
                utilization * 100.0,
                bond.damage,
                stress.tensile_pa / 1.0e6,
                stress.compressive_pa / 1.0e6,
                stress.shear_pa / 1.0e6,
                stress.bending_pa / 1.0e6,
                stress.torsional_pa / 1.0e6
            );
        }
    }
}
#[cfg(test)]
mod tests {
    use super::{BondStress, calculate_bond_stress};
    use crate::config::BondStrengthDefinition;
    use bevy::prelude::Vec3;

    fn strength() -> BondStrengthDefinition {
        BondStrengthDefinition {
            elastic_modulus_pa: 1.0e9,
            tensile_strength_pa: 10.0,
            compressive_strength_pa: 15.0,
            shear_strength_pa: 10.0,
            bending_strength_pa: 10.0,
            torsional_strength_pa: 10.0,
            damage_rate_per_s: 1.0,
        }
    }

    #[test]
    fn axial_tension_uses_face_area_and_does_not_invent_shear() {
        let stress = calculate_bond_stress(Vec3::Y * 80.0, Vec3::ZERO, Vec3::Y, 2.0);
        assert!((stress.tensile_pa - 20.0).abs() < 1.0e-5);
        assert_eq!(stress.compressive_pa, 0.0);
        assert_eq!(stress.shear_pa, 0.0);
        assert_eq!(stress.bending_pa, 0.0);
    }

    #[test]
    fn axial_compression_uses_its_own_material_limit() {
        let stress = calculate_bond_stress(Vec3::NEG_Y * 80.0, Vec3::ZERO, Vec3::Y, 2.0);
        assert_eq!(stress.tensile_pa, 0.0);
        assert!((stress.compressive_pa - 20.0).abs() < 1.0e-5);
        assert!(stress.exceeds(&strength()));
    }

    #[test]
    fn shear_and_bending_modes_are_independent_of_axial_tension() {
        let shear = calculate_bond_stress(Vec3::X * 40.0, Vec3::ZERO, Vec3::Y, 2.0);
        assert_eq!(shear.tensile_pa, 0.0);
        assert_eq!(shear.compressive_pa, 0.0);
        assert!((shear.shear_pa - 10.0).abs() < 1.0e-5);

        let bending = calculate_bond_stress(Vec3::ZERO, Vec3::X * (10.0 / 6.0), Vec3::Y, 1.0);
        assert!((bending.bending_pa - 10.0).abs() < 1.0e-5);
    }

    #[test]
    fn bond_failure_is_driven_by_mode_specific_strength_not_impact_speed() {
        let stress = BondStress {
            tensile_pa: 11.0,
            ..Default::default()
        };
        assert!(stress.exceeds(&strength()));
    }

    #[test]
    fn invalid_geometry_fails_closed() {
        let stress = calculate_bond_stress(Vec3::X, Vec3::Y, Vec3::Z, f32::NAN);
        assert!(stress.tensile_pa.is_infinite());
        assert!(stress.compressive_pa.is_infinite());
    }
}

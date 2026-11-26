use avian3d::prelude::{FixedJoint, JointCollisionDisabled, JointForces};
use bevy::prelude::*;

use crate::config::BondStrengthDefinition;

#[derive(Component, Clone, Debug)]
pub struct VoxelBond {
    pub id: String,
    pub body1: Entity,
    pub body2: Entity,
    pub normal_local: Vec3,
    pub face_size_m: f32,
    pub strength: BondStrengthDefinition,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BondStress {
    pub tensile_pa: f32,
    pub shear_pa: f32,
    pub bending_pa: f32,
    pub torsional_pa: f32,
}

impl BondStress {
    pub fn exceeds(self, strength: &BondStrengthDefinition) -> bool {
        self.tensile_pa > strength.tensile_strength_pa
            || self.shear_pa > strength.shear_strength_pa
            || self.bending_pa > strength.bending_strength_pa
            || self.torsional_pa > strength.torsional_strength_pa
    }
}

/// Converts Avian's joint resultant into nominal face stresses.
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
        shear_pa: shear_force.length() / area,
        bending_pa: 6.0 * bending_moment.length() / cube,
        torsional_pa: 3.0 * axial_moment.abs() / cube,
    }
}

pub fn spawn_voxel_bond(
    commands: &mut Commands,
    id: String,
    body1: Entity,
    body2: Entity,
    local_anchor1: Vec3,
    local_anchor2: Vec3,
    normal_local: Vec3,
    face_size_m: f32,
    strength: BondStrengthDefinition,
) -> Entity {
    commands
        .spawn((
            Name::new(id.clone()),
            FixedJoint::new(body1, body2)
                .with_local_anchor1(local_anchor1)
                .with_local_anchor2(local_anchor2),
            JointForces::new(),
            JointCollisionDisabled,
            VoxelBond {
                id,
                body1,
                body2,
                normal_local,
                face_size_m,
                strength,
            },
        ))
        .id()
}

pub(super) fn break_overstressed_bonds(
    mut commands: Commands,
    bonds: Query<(Entity, &VoxelBond, &JointForces)>,
    transforms: Query<&Transform>,
    mut journal: ResMut<crate::physics::DestructionJournal>,
) {
    for (joint_entity, bond, joint_forces) in &bonds {
        if journal.broken_bonds.contains(&bond.id) {
            commands.entity(joint_entity).despawn();
            continue;
        }

        let Ok(body_transform) = transforms.get(bond.body1) else {
            commands.entity(joint_entity).despawn();
            journal.broken_bonds.insert(bond.id.clone());
            continue;
        };

        let normal_world = body_transform.rotation * bond.normal_local;
        let stress = calculate_bond_stress(
            joint_forces.force(),
            joint_forces.torque(),
            normal_world,
            bond.face_size_m,
        );

        if stress.exceeds(&bond.strength) {
            journal.broken.insert(bond.id.clone());
            commands.entity(joint_entity).despawn();
            info!(
                "Voxel bond '{}' failed: tensile {:.2} MPa, shear {:.2} MPa, bending {:.2} MPa, torsion {:.2} MPa",
                bond.id,
                stress.tensile_pa / 1.0e6,
                stress.shear_pa / 1.0e6,
                stress.bending_pa / 1.0e6,
                stress.torsional_pa / 1.0e6
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{calculate_bond_stress, BondStress};
    use crate::config::BondStrengthDefinition;
    use bevy::prelude::Vec3;

    fn strength() -> BondStrengthDefinition {
        BondStrengthDefinition {
            tensile_strength_pa: 10.0,
            shear_strength_pa: 10.0,
            bending_strength_pa: 10.0,
            torsional_strength_pa: 10.0,
        }
    }

    #[test]
    fn axial_tension_uses_face_area_and_does_not_invent_shear() {
        let stress = calculate_bond_stress(Vec3::Y * 80.0, Vec3::ZERO, Vec3::Y, 2.0);
        assert!((stress.tensile_pa - 20.0).abs() < 1.0e-5);
        assert_eq!(stress.shear_pa, 0.0);
        assert_eq!(stress.bending_pa, 0.0);
    }

    #[test]
    fn shear_and_bending_modes_are_independent_of_axial_tension() {
        let shear = calculate_bond_stress(Vec3::X * 40.0, Vec3::ZERO, Vec3::Y, 2.0);
        assert_eq!(shear.tensile_pa, 0.0);
        assert!((shear.shear_pa - 10.0).abs() < 1.0e-5);

        let bending = calculate_bond_stress(
            Vec3::ZERO,
            Vec3::X * (10.0 / 6.0),
            Vec3::Y,
            1.0,
        );
        assert!((bending.bending_pa - 10.0).abs() < 1.0e-5);
    }

    #[test]
    fn bond_failure_is_driven_by_mode_specific_strength_not_impact_speed() {
        let stress = BondStress {
            tensile_pa: 11.0,
            shear_pa: 0.0,
            bending_pa: 0.0,
            torsional_pa: 0.0,
        };
        assert!(stress.exceeds(&strength()));
    }

    #[test]
    fn invalid_geometry_fails_closed() {
        let stress = calculate_bond_stress(
            Vec3::X,
            Vec3::Y,
            Vec3::Z,
            f32::NAN,
        );
        assert!(stress.tensile_pa.is_infinite());
    }
}

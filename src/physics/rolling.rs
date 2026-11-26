use std::collections::HashMap;

use avian3d::prelude::{AngularVelocity, Collisions};
use bevy::prelude::*;

use crate::config::ColliderShape;

/// Contact-only rolling resistance, with principal moments of inertia in body-local space.
///
/// This is a Coulomb-style rolling-resistance impulse model. It acts only on the component of
/// angular velocity tangent to a supporting contact normal; spin about the normal is left alone.
#[derive(Component, Clone, Copy, Debug)]
pub struct RollingResistance {
    pub coefficient: f32,
    pub effective_radius_m: f32,
    pub inertia_local_kg_m2: Vec3,
}

impl RollingResistance {
    pub fn from_collider(
        shape: &ColliderShape,
        density_kg_m3: f32,
        coefficient: f32,
        scale: f32,
    ) -> Option<Self> {
        if !coefficient.is_finite()
            || coefficient <= 0.0
            || !density_kg_m3.is_finite()
            || density_kg_m3 <= 0.0
            || !scale.is_finite()
            || scale <= 0.0
        {
            return None;
        }

        let (mass, radius, inertia) = match shape {
            ColliderShape::Cuboid { half_extents } => {
                let half = Vec3::from_array(*half_extents) * scale;
                let size = half * 2.0;
                let mass = size.x * size.y * size.z * density_kg_m3;
                let inertia = Vec3::new(
                    mass * (half.y * half.y + half.z * half.z) / 3.0,
                    mass * (half.x * half.x + half.z * half.z) / 3.0,
                    mass * (half.x * half.x + half.y * half.y) / 3.0,
                );
                (mass, half.min_element(), inertia)
            }
            ColliderShape::Sphere { radius } => {
                let radius = radius * scale;
                let mass = (4.0 / 3.0) * std::f32::consts::PI * radius.powi(3) * density_kg_m3;
                let moment = 0.4 * mass * radius * radius;
                (mass, radius, Vec3::splat(moment))
            }
            ColliderShape::Capsule {
                radius,
                segment_length,
            } => {
                let radius = radius * scale;
                let length = segment_length * scale;
                let cylinder_mass = std::f32::consts::PI * radius * radius * length * density_kg_m3;
                let cap_mass = (4.0 / 3.0) * std::f32::consts::PI * radius.powi(3) * density_kg_m3;
                let mass = cylinder_mass + cap_mass;
                // Approximate the two hemispherical caps as a centred sphere. The axial
                // moment is exact for that decomposition; the transverse moment is conservative.
                let axial =
                    0.5 * cylinder_mass * radius * radius + 0.4 * cap_mass * radius * radius;
                let transverse = cylinder_mass * (3.0 * radius * radius + length * length) / 12.0
                    + cap_mass * (0.4 * radius * radius + 0.25 * length * length);
                (mass, radius, Vec3::new(transverse, axial, transverse))
            }
        };

        if !mass.is_finite()
            || mass <= 0.0
            || !radius.is_finite()
            || radius <= 0.0
            || !inertia.is_finite()
            || inertia.min_element() <= 0.0
        {
            return None;
        }

        Some(Self {
            coefficient,
            effective_radius_m: radius,
            inertia_local_kg_m2: inertia,
        })
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct ContactSupport {
    weighted_normal: Vec3,
    normal_impulse_ns: f32,
}

pub(super) fn apply_contact_rolling_resistance(
    collisions: Collisions,
    mut bodies: Query<(&RollingResistance, &Transform, &mut AngularVelocity)>,
) {
    let mut supports = HashMap::<Entity, ContactSupport>::new();

    for pair in collisions.iter() {
        let mut pair_normal = Vec3::ZERO;
        let mut pair_impulse = 0.0_f32;

        for manifold in &pair.manifolds {
            let impulse = manifold
                .points
                .iter()
                .map(|point| point.normal_impulse.max(0.0))
                .sum::<f32>();
            if impulse <= 0.0 {
                continue;
            }
            pair_normal += manifold.normal * impulse;
            pair_impulse += impulse;
        }

        if pair_impulse <= 0.0 {
            continue;
        }

        let normal = pair_normal.normalize_or_zero();
        if normal == Vec3::ZERO {
            continue;
        }

        supports
            .entry(pair.collider1)
            .and_modify(|support| {
                support.weighted_normal += normal * pair_impulse;
                support.normal_impulse_ns += pair_impulse;
            })
            .or_insert(ContactSupport {
                weighted_normal: normal * pair_impulse,
                normal_impulse_ns: pair_impulse,
            });
        supports
            .entry(pair.collider2)
            .and_modify(|support| {
                support.weighted_normal -= normal * pair_impulse;
                support.normal_impulse_ns += pair_impulse;
            })
            .or_insert(ContactSupport {
                weighted_normal: -normal * pair_impulse,
                normal_impulse_ns: pair_impulse,
            });
    }

    for (entity, support) in supports {
        let Ok((rolling, transform, mut angular_velocity)) = bodies.get_mut(entity) else {
            continue;
        };

        let world_normal = support.weighted_normal.normalize_or_zero();
        if world_normal == Vec3::ZERO {
            continue;
        }

        let local_normal = transform.rotation.inverse() * world_normal;
        let local_omega = transform.rotation.inverse() * angular_velocity.0;
        let new_local_omega = dissipate_rolling_spin(
            local_omega,
            local_normal,
            rolling.inertia_local_kg_m2,
            rolling.coefficient * rolling.effective_radius_m * support.normal_impulse_ns,
        );

        angular_velocity.0 = transform.rotation * new_local_omega;
    }
}

/// Applies an angular impulse opposite rolling angular momentum, capped before it can reverse
/// any principal-axis spin component. Returns the updated body-local angular velocity.
fn dissipate_rolling_spin(
    local_omega: Vec3,
    local_normal: Vec3,
    inertia: Vec3,
    angular_impulse_ns_m: f32,
) -> Vec3 {
    if !local_omega.is_finite()
        || !local_normal.is_finite()
        || !inertia.is_finite()
        || inertia.min_element() <= 0.0
        || !angular_impulse_ns_m.is_finite()
        || angular_impulse_ns_m <= 0.0
    {
        return local_omega;
    }

    let normal = local_normal.normalize_or_zero();
    if normal == Vec3::ZERO {
        return local_omega;
    }

    let rolling_omega = local_omega - normal * local_omega.dot(normal);
    let rolling_speed = rolling_omega.length();
    if rolling_speed <= 1.0e-8 {
        return local_omega;
    }

    // Apply torque opposite the tangential angular velocity. The impulse direction is
    // perpendicular to the contact normal in body-local space; the inertia tensor converts
    // that torque impulse into angular velocity, including anisotropic-axis coupling.
    let torque_axis = rolling_omega / rolling_speed;
    let inverse_inertia_torque = torque_axis / inertia;
    let effective_inverse_inertia = torque_axis.dot(inverse_inertia_torque);
    if !effective_inverse_inertia.is_finite() || effective_inverse_inertia <= 0.0 {
        return local_omega;
    }

    // For delta-L = -j * torque_axis:
    // delta-E = -j * |omega_t| + 0.5 * j^2 * (axis . I^-1 . axis).
    // This cap guarantees non-increasing rotational kinetic energy.
    let energy_safe_impulse = rolling_speed / effective_inverse_inertia;

    // Avoid reversing any principal-axis component in one contact step. This can be
    // stricter than the energy bound for strongly anisotropic bodies.
    let mut no_reversal_impulse = f32::INFINITY;
    for axis in 0..3 {
        if torque_axis[axis] * local_omega[axis] > 0.0 {
            let limit = local_omega[axis].abs() * inertia[axis] / torque_axis[axis].abs();
            no_reversal_impulse = no_reversal_impulse.min(limit);
        }
    }

    let applied_impulse = angular_impulse_ns_m
        .min(energy_safe_impulse)
        .min(no_reversal_impulse);
    if !applied_impulse.is_finite() || applied_impulse <= 0.0 {
        return local_omega;
    }

    local_omega - inverse_inertia_torque * applied_impulse
    result
}

#[cfg(test)]
mod tests {
    use super::{RollingResistance, dissipate_rolling_spin};
    use crate::config::ColliderShape;
    use bevy::prelude::Vec3;

    fn rotational_energy(omega: Vec3, inertia: Vec3) -> f32 {
        0.5 * (inertia * omega).dot(omega)
    }

    #[test]
    fn rolling_impulse_never_increases_rotational_energy_or_reverses_spin() {
        let inertia = Vec3::new(0.8, 1.1, 1.4);
        for omega in [
            Vec3::new(2.0, 0.5, -3.0),
            Vec3::new(-1.0, 4.0, 0.25),
            Vec3::new(0.0, 0.0, 5.0),
        ] {
            let normal = Vec3::Y;
            let result = dissipate_rolling_spin(omega, normal, inertia, 0.7);
            assert!(
                rotational_energy(result, inertia) <= rotational_energy(omega, inertia) + 1.0e-5
            );
            for axis in 0..3 {
                assert!(
                    result[axis] == 0.0 || result[axis] * omega[axis] >= 0.0,
                    "rolling torque reversed spin component {axis}"
                );
            }
            assert!((result.dot(normal) - omega.dot(normal)).abs() < 1.0e-5);
        }
    }

    #[test]
    fn oblique_contact_impulse_preserves_energy_and_has_no_normal_torque() {
        let inertia = Vec3::new(0.4, 1.7, 3.1);
        let omega = Vec3::new(3.0, -1.0, 2.0);
        let normal = Vec3::new(1.0, 2.0, 3.0).normalize();
        let result = dissipate_rolling_spin(omega, normal, inertia, 1.25);
        assert!(
            rotational_energy(result, inertia)
                <= rotational_energy(omega, inertia) + 1.0e-5
        );

        let angular_impulse = inertia * (result - omega);
        assert!(angular_impulse.dot(normal).abs() < 1.0e-5);
    }

    #[test]
    fn rolling_resistance_is_zero_for_a_body_spinning_only_about_contact_normal() {
        let omega = Vec3::Y * 8.0;
        assert_eq!(
            dissipate_rolling_spin(omega, Vec3::Y, Vec3::ONE, 0.5),
            omega
        );
    }

    #[test]
    fn collider_inertia_uses_density_and_si_dimensions() {
        let rolling = RollingResistance::from_collider(
            &ColliderShape::Sphere { radius: 0.5 },
            1000.0,
            0.02,
            1.0,
        )
        .expect("valid sphere rolling data should be constructed");
        let mass = (4.0 / 3.0) * std::f32::consts::PI * 0.5_f32.powi(3) * 1000.0;
        assert!((rolling.inertia_local_kg_m2.x - 0.4 * mass * 0.25).abs() < 1.0e-3);
    }
}

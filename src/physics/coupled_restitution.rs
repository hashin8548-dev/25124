//! Contact-wide projected Gauss-Seidel restitution for coupled rigid-body impacts.
//!
//! Avian 0.8 applies restitution per contact constraint. Constraints that share a body can
//! therefore undo one another's restitution impulses. This pass iterates the active contact
//! graph as one coupled system after Avian has finalized velocities and stored contact impulses.
//! It corrects only the normal component; friction remains Avian's contact-solver responsibility.

use std::collections::{HashMap, HashSet};

use avian3d::{
    dynamics::solver::SolverConfig,
    prelude::{
        AngularVelocity, ColliderOf, ComputedAngularInertia, ComputedMass, ContactGraph,
        LinearVelocity, PhysicsLengthUnit, RigidBody, Rotation,
    },
};
use bevy::prelude::*;

#[derive(Clone, Copy)]
struct BodyMotion {
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    inverse_mass: f32,
    angular_inertia: Option<ComputedAngularInertia>,
    rotation: Quat,
}

impl BodyMotion {
    fn inverse_inertia_mul(self, vector: Vec3) -> Vec3 {
        self.angular_inertia.map_or(Vec3::ZERO, |inertia| {
            inertia.rotated(self.rotation).inverse() * vector
        })
    }

    fn apply_impulse(&mut self, arm: Vec3, impulse: Vec3, sign: f32) {
        self.linear_velocity += impulse * (sign * self.inverse_mass);
        if self.angular_inertia.is_some() {
            self.angular_velocity += self.inverse_inertia_mul(arm.cross(impulse)) * sign;
        }
    }

    fn point_velocity(self, arm: Vec3) -> Vec3 {
        self.linear_velocity + self.angular_velocity.cross(arm)
    }
}

type RigidBodyMotionQuery<'world, 'state> = Query<
    'world,
    'state,
    (
        &'static RigidBody,
        Option<&'static ComputedMass>,
        Option<&'static ComputedAngularInertia>,
        Option<&'static Rotation>,
        Option<&'static mut LinearVelocity>,
        Option<&'static mut AngularVelocity>,
    ),
>;

#[derive(Clone, Copy)]
struct ContactPointConstraint {
    pair_index: usize,
    manifold_index: usize,
    point_index: usize,
    body1: Entity,
    body2: Entity,
    normal: Vec3,
    anchor1: Vec3,
    anchor2: Vec3,
    initial_normal_speed: f32,
    restitution: f32,
    friction: f32,
    accumulated_normal_impulse: f32,
}

/// Iteratively solves restitution across contacts that share rigid bodies.
///
/// Scheduled after Avian's `StoreContactImpulses` stage: body velocities already include
/// Avian's solver and restitution response, while ContactGraph exposes each contact's
/// accumulated normal impulse and pre-solve approach velocity.
pub(super) fn solve_coupled_restitution(
    mut contacts: ResMut<ContactGraph>,
    solver_config: Res<SolverConfig>,
    length_unit: Res<PhysicsLengthUnit>,
    collider_owners: Query<&ColliderOf>,
    mut bodies: RigidBodyMotionQuery<'_, '_>,
) {
    let threshold = solver_config.restitution_threshold * length_unit.0;
    if solver_config.restitution_iterations == 0 || threshold < 0.0 {
        return;
    }

    let mut constraints = Vec::new();
    let mut body_entities = HashSet::new();

    for (pair_index, pair) in contacts.active_pairs().iter().enumerate() {
        let body1 = collider_owners
            .get(pair.collider1)
            .map_or(pair.collider1, |owner| owner.body);
        let body2 = collider_owners
            .get(pair.collider2)
            .map_or(pair.collider2, |owner| owner.body);

        if body1 == body2 {
            continue;
        }

        for (manifold_index, manifold) in pair.manifolds.iter().enumerate() {
            if !manifold.restitution.is_finite()
                || manifold.restitution <= 0.0
                || !manifold.normal.is_finite()
            {
                continue;
            }
            let normal = manifold.normal.normalize_or_zero();
            if normal == Vec3::ZERO {
                continue;
            }

            for (point_index, point) in manifold.points.iter().enumerate() {
                if !point.normal_speed.is_finite()
                    || point.normal_speed >= -threshold
                    || !point.normal_impulse.is_finite()
                    || point.normal_impulse <= 0.0
                {
                    continue;
                }

                constraints.push(ContactPointConstraint {
                    pair_index,
                    manifold_index,
                    point_index,
                    body1,
                    body2,
                    normal,
                    anchor1: point.anchor1,
                    anchor2: point.anchor2,
                    initial_normal_speed: point.normal_speed,
                    restitution: manifold.restitution.clamp(0.0, 1.0),
                    friction: manifold.friction.max(0.0),
                    accumulated_normal_impulse: point.normal_impulse.max(0.0),
                });
                body_entities.insert(body1);
                body_entities.insert(body2);
            }
        }
    }

    if constraints.is_empty() {
        return;
    }

    let mut motions = HashMap::with_capacity(body_entities.len());
    for entity in body_entities {
        let Ok((rigid_body, mass, angular_inertia, rotation, linear_velocity, angular_velocity)) =
            bodies.get_mut(entity)
        else {
            motions.insert(
                entity,
                BodyMotion {
                    linear_velocity: Vec3::ZERO,
                    angular_velocity: Vec3::ZERO,
                    inverse_mass: 0.0,
                    angular_inertia: None,
                    rotation: Quat::IDENTITY,
                },
            );
            continue;
        };

        let dynamic = matches!(*rigid_body, RigidBody::Dynamic);
        let inverse_mass = if dynamic && linear_velocity.is_some() {
            mass.map_or(0.0, |mass| mass.inverse())
        } else {
            0.0
        };
        let angular_inertia = (dynamic && angular_velocity.is_some())
            .then(|| angular_inertia.copied())
            .flatten();

        motions.insert(
            entity,
            BodyMotion {
                linear_velocity: linear_velocity
                    .as_deref()
                    .map_or(Vec3::ZERO, |velocity| velocity.0),
                angular_velocity: angular_velocity
                    .as_deref()
                    .map_or(Vec3::ZERO, |velocity| velocity.0),
                inverse_mass,
                angular_inertia,
                rotation: rotation.map_or(Quat::IDENTITY, |rotation| rotation.0),
            },
        );
    }

    solve_contact_set(
        &mut motions,
        &mut constraints,
        solver_config.restitution_iterations,
    );

    // Keep Avian's next-frame warm-start impulses coherent with the corrected velocities.
    let active_pairs = contacts.active_pairs_mut();
    for constraint in &constraints {
        let point = &mut active_pairs[constraint.pair_index].manifolds[constraint.manifold_index]
            .points[constraint.point_index];
        point.normal_impulse = constraint.accumulated_normal_impulse;
        point.warm_start_normal_impulse = constraint.accumulated_normal_impulse;

        // Friction warm starts must remain inside the updated Coulomb friction cone.
        let tangent_limit = constraint.friction * constraint.accumulated_normal_impulse;
        let tangent_magnitude = point.warm_start_tangent_impulse.length();
        if tangent_magnitude > tangent_limit && tangent_magnitude > f32::EPSILON {
            point.warm_start_tangent_impulse *= tangent_limit / tangent_magnitude;
        }
    }

    for (entity, motion) in motions {
        if motion.inverse_mass == 0.0 && motion.angular_inertia.is_none() {
            continue;
        }
        if let Ok((_, _, _, _, linear_velocity, angular_velocity)) = bodies.get_mut(entity) {
            if let Some(mut velocity) = linear_velocity {
                velocity.0 = motion.linear_velocity;
            }
            if let Some(mut velocity) = angular_velocity {
                velocity.0 = motion.angular_velocity;
            }
        }
    }
}

fn solve_contact_set(
    bodies: &mut HashMap<Entity, BodyMotion>,
    contacts: &mut [ContactPointConstraint],
    iterations: usize,
) {
    for _ in 0..iterations {
        for contact in contacts.iter_mut() {
            let Some(body1) = bodies.get(&contact.body1).copied() else {
                continue;
            };
            let Some(body2) = bodies.get(&contact.body2).copied() else {
                continue;
            };

            let arm1_normal = contact.anchor1.cross(contact.normal);
            let arm2_normal = contact.anchor2.cross(contact.normal);
            let effective_inverse_mass = body1.inverse_mass
                + body2.inverse_mass
                + arm1_normal.dot(body1.inverse_inertia_mul(arm1_normal))
                + arm2_normal.dot(body2.inverse_inertia_mul(arm2_normal));

            if !effective_inverse_mass.is_finite() || effective_inverse_mass <= 1.0e-8 {
                continue;
            }

            let relative_normal_speed = (body2.point_velocity(contact.anchor2)
                - body1.point_velocity(contact.anchor1))
            .dot(contact.normal);
            let target_separation_speed = -contact.restitution * contact.initial_normal_speed;
            let impulse_delta =
                (target_separation_speed - relative_normal_speed) / effective_inverse_mass;
            let next_impulse = (contact.accumulated_normal_impulse + impulse_delta).max(0.0);
            let applied_impulse = next_impulse - contact.accumulated_normal_impulse;

            if !applied_impulse.is_finite() || applied_impulse.abs() <= 1.0e-8 {
                continue;
            }

            contact.accumulated_normal_impulse = next_impulse;
            let impulse = contact.normal * applied_impulse;
            if let Some(body1) = bodies.get_mut(&contact.body1) {
                body1.apply_impulse(contact.anchor1, impulse, -1.0);
            }
            if let Some(body2) = bodies.get_mut(&contact.body2) {
                body2.apply_impulse(contact.anchor2, impulse, 1.0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{BodyMotion, ContactPointConstraint, solve_contact_set};
    use bevy::prelude::{Entity, Quat, Vec3};
    use std::collections::HashMap;

    fn motion(velocity: f32) -> BodyMotion {
        BodyMotion {
            linear_velocity: Vec3::X * velocity,
            angular_velocity: Vec3::ZERO,
            inverse_mass: 1.0,
            angular_inertia: None,
            rotation: Quat::IDENTITY,
        }
    }

    fn contact(body1: Entity, body2: Entity) -> ContactPointConstraint {
        ContactPointConstraint {
            pair_index: 0,
            manifold_index: 0,
            point_index: 0,
            body1,
            body2,
            normal: Vec3::X,
            anchor1: Vec3::ZERO,
            anchor2: Vec3::ZERO,
            initial_normal_speed: -2.0,
            restitution: 1.0,
            friction: 0.0,
            accumulated_normal_impulse: 0.0,
        }
    }

    #[test]
    fn bilateral_three_body_contacts_converge_to_elastic_rebound() {
        let left = Entity::from_bits(1);
        let middle = Entity::from_bits(2);
        let right = Entity::from_bits(3);
        let mut bodies = HashMap::from([
            (left, motion(2.0)),
            (middle, motion(0.0)),
            (right, motion(-2.0)),
        ]);
        let mut contacts = [contact(left, middle), contact(middle, right)];

        solve_contact_set(&mut bodies, &mut contacts, 16);

        let left_velocity = bodies[&left].linear_velocity.x;
        let middle_velocity = bodies[&middle].linear_velocity.x;
        let right_velocity = bodies[&right].linear_velocity.x;
        assert!((left_velocity + 2.0).abs() < 0.001, "{left_velocity}");
        assert!(middle_velocity.abs() < 0.001, "{middle_velocity}");
        assert!((right_velocity - 2.0).abs() < 0.001, "{right_velocity}");

        let momentum = left_velocity + middle_velocity + right_velocity;
        let energy =
            0.5 * (left_velocity.powi(2) + middle_velocity.powi(2) + right_velocity.powi(2));
        assert!(momentum.abs() < 0.001, "momentum error {momentum}");
        assert!(
            (energy - 4.0).abs() < 0.002,
            "energy error {}",
            energy - 4.0
        );
    }

    #[test]
    fn one_body_contact_chain_does_not_create_energy() {
        let first = Entity::from_bits(11);
        let second = Entity::from_bits(12);
        let mut bodies = HashMap::from([(first, motion(1.0)), (second, motion(0.0))]);
        let mut single_contact = contact(first, second);
        single_contact.initial_normal_speed = -1.0;
        let mut contacts = [single_contact];
        solve_contact_set(&mut bodies, &mut contacts, 16);
        let energy = 0.5
            * (bodies[&first].linear_velocity.length_squared()
                + bodies[&second].linear_velocity.length_squared());
        assert!(
            (energy - 0.5).abs() < 0.001,
            "energy error {}",
            energy - 0.5
        );
    }
}

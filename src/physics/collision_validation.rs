use std::time::Duration;

use avian3d::{
    dynamics::solver::SolverConfig,
    dynamics::solver::schedule::SolverSystems,
    prelude::{
        Collider, ComputedMass, ContactGraph, Friction, Gravity, LinearVelocity, Mass,
        PhysicsPlugins, PhysicsSchedule, PhysicsStepSystems, Restitution, RigidBody, SubstepCount,
    },
};
use bevy::{prelude::*, time::TimeUpdateStrategy};

#[derive(Component, Clone, Copy)]
struct CollisionParticle(usize);

#[derive(Resource, Clone, Copy, Default, Debug)]
struct ContactProbe {
    max_restitution: f32,
    min_approach_speed: f32,
    max_approach_speed: f32,
    max_normal_impulse: f32,
    max_contact_count: usize,
}

fn test_app() -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        PhysicsPlugins::default(),
        bevy::asset::AssetPlugin::default(),
        bevy::mesh::MeshPlugin,
    ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
        1.0 / 120.0,
    )))
    .insert_resource(Time::<Fixed>::from_hz(120.0))
    .insert_resource(SubstepCount(16))
    .insert_resource(Gravity::ZERO)
    .init_resource::<ContactProbe>()
    .insert_resource(SolverConfig {
        contact_damping_ratio: 10.0,
        contact_frequency_factor: 1.5,
        max_overlap_solve_speed: 1.0,
        warm_start_coefficient: 1.0,
        restitution_threshold: 1.0,
        restitution_iterations: 16,
    });
    app.add_systems(
        PhysicsSchedule,
        super::rolling::apply_contact_rolling_resistance
            .after(PhysicsStepSystems::NarrowPhase)
            .before(PhysicsStepSystems::Solver),
    );
    app.add_systems(
        PhysicsSchedule,
        super::coupled_restitution::solve_coupled_restitution
            .in_set(PhysicsStepSystems::Solver)
            .after(SolverSystems::StoreContactImpulses),
    );
    app.add_systems(Update, capture_contact_probe);
    app.finish();
    app
}

fn spawn_particle_chain(mut commands: Commands, positions: [f32; 3], velocities: [f32; 3]) {
    for index in 0..3 {
        commands.spawn((
            CollisionParticle(index),
            Transform::from_xyz(positions[index], 0.0, 0.0),
            RigidBody::Dynamic,
            Collider::sphere(0.5),
            Mass(1.0),
            LinearVelocity(Vec3::X * velocities[index]),
            Restitution::new(1.0),
            Friction::new(0.0),
        ));
    }
}

fn capture_contact_probe(graph: Res<ContactGraph>, mut probe: ResMut<ContactProbe>) {
    for pair in graph.active_pairs() {
        for manifold in &pair.manifolds {
            probe.max_restitution = probe.max_restitution.max(manifold.restitution);
            for point in &manifold.points {
                probe.min_approach_speed = probe.min_approach_speed.min(point.normal_speed);
                probe.max_approach_speed = probe.max_approach_speed.max(point.normal_speed.abs());
                probe.max_normal_impulse = probe.max_normal_impulse.max(point.normal_impulse);
                probe.max_contact_count += 1;
            }
        }
    }
}
fn simulate(app: &mut App, steps: usize) {
    for _ in 0..steps {
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 120.0,
        )));
        app.update();
    }
}

fn conserved_totals(app: &mut App) -> (Vec3, f32, [Vec3; 3]) {
    let mut query = app
        .world_mut()
        .query::<(&CollisionParticle, &ComputedMass, &LinearVelocity)>();
    let mut momentum = Vec3::ZERO;
    let mut energy = 0.0;
    let mut velocities = [Vec3::ZERO; 3];
    for (particle, mass, velocity) in query.iter(app.world()) {
        let body_mass = mass.value();
        momentum += velocity.0 * body_mass;
        energy += 0.5 * body_mass * velocity.0.length_squared();
        velocities[particle.0] = velocity.0;
    }
    (momentum, energy, velocities)
}

fn run_chain(positions: [f32; 3], velocities: [f32; 3]) -> (Vec3, f32, [Vec3; 3], ContactProbe) {
    let mut app = test_app();
    app.add_systems(Startup, move |commands: Commands| {
        spawn_particle_chain(commands, positions, velocities);
    });
    app.update();
    simulate(&mut app, 240);
    let totals = conserved_totals(&mut app);
    let probe = *app.world().resource::<ContactProbe>();
    (totals.0, totals.1, totals.2, probe)
}

fn assert_conserved(
    before_momentum: Vec3,
    before_energy: f32,
    after: (Vec3, f32, [Vec3; 3], ContactProbe),
) {
    let (after_momentum, after_energy, _, probe) = after;
    let momentum_error = (after_momentum - before_momentum).length();
    let energy_relative_error = (after_energy - before_energy).abs() / before_energy.max(1.0e-6);
    assert!(
        (probe.max_restitution - 1.0).abs() < 1.0e-6,
        "contact restitution was not 1.0: {probe:?}"
    );
    assert!(
        probe.min_approach_speed < -1.0,
        "test did not capture an approaching impact: {probe:?}"
    );
    assert!(
        probe.max_normal_impulse > 0.0,
        "solver applied no contact impulse: {probe:?}"
    );
    assert!(
        momentum_error <= 0.02,
        "linear momentum drifted by {momentum_error:.6} kg*m/s"
    );
    assert!(
        energy_relative_error <= 0.05,
        "kinetic energy drifted by {:.3}% (before {before_energy:.5}, after {after_energy:.5}, velocities {after:?}, contact probe {probe:?})",
        energy_relative_error * 100.0
    );
}

#[test]
fn three_body_collision_chain_conserves_momentum_and_kinetic_energy() {
    let (momentum, energy, velocities, probe) = run_chain([-1.5, 0.0, 1.5], [3.0, 0.0, 0.0]);
    assert_conserved(Vec3::X * 3.0, 4.5, (momentum, energy, velocities, probe));
    assert!(
        velocities[2].x > 2.8,
        "the final sphere should receive the elastic impact; velocities: {velocities:?}"
    );
}

fn run_box_pair() -> (Vec3, f32, [Vec3; 3], ContactProbe) {
    let mut app = test_app();
    app.add_systems(Startup, |mut commands: Commands| {
        for (index, x, velocity) in [(0, -0.75, 2.0), (1, 0.75, -2.0)] {
            commands.spawn((
                CollisionParticle(index),
                Transform::from_xyz(x, 0.0, 0.0),
                RigidBody::Dynamic,
                Collider::cuboid(1.0, 1.0, 1.0),
                Mass(1.0),
                LinearVelocity(Vec3::X * velocity),
                Restitution::new(1.0),
                Friction::new(0.0),
            ));
        }
    });
    app.update();
    simulate(&mut app, 120);
    let totals = conserved_totals(&mut app);
    let probe = *app.world().resource::<ContactProbe>();
    (totals.0, totals.1, totals.2, probe)
}

#[test]
fn multi_point_box_collision_conserves_momentum_and_kinetic_energy() {
    let result = run_box_pair();
    let (_, _, velocities, probe) = result;
    assert_conserved(Vec3::ZERO, 4.0, result);
    assert!(
        probe.max_contact_count >= 4,
        "expected a face-to-face manifold with multiple contact points: {probe:?}"
    );
    assert!(
        velocities[0].x < -1.8 && velocities[1].x > 1.8,
        "aligned boxes should exchange their velocities: {velocities:?}"
    );
}

#[test]
fn symmetric_three_body_impact_preserves_zero_net_momentum_and_energy() {
    let (momentum, energy, velocities, probe) = run_chain([-1.5, 0.0, 1.5], [2.0, 0.0, -2.0]);
    assert_conserved(Vec3::ZERO, 4.0, (momentum, energy, velocities, probe));
    assert!(
        velocities[0].x < -1.8 && velocities[2].x > 1.8,
        "symmetric outer bodies should rebound symmetrically: {velocities:?}"
    );
}

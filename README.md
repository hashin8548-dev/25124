# Physics Sandbox

A data-driven 3D rigid-body and voxel sandbox built with Rust, Bevy 0.20.1, and Avian 0.8.0. The executable hosts reusable simulation systems; the simulation, physical materials, object definitions, world composition, chunk data, and menu layout live outside `src/`.

## Portable project layout

```text
physics_sandbox.exe
data/
  engine.json                 Fixed-step solver, sleeping, camera, lighting
  materials.json              SI density, friction and restitution catalog
  components.json             Reusable collider, visual and damage definitions
assets/
  ui/main_menu.json           Main-menu text, layout, colors and actions
  models/ ...                 Optional glTF models and textures
mods/                         Reserved for content packages
worlds/
  <world>/
    world.json                World metadata and region references
    regions/
      <region>.json           Region index and chunk references
      chunks/
        <chunk>.json          Palette-indexed voxel cells
    state.json                Generated runtime save, not authored world content
```

The Rust source owns systems and validation, not scenario-specific visuals or UI markup. New physical objects are defined in `data/components.json`, with physical properties referencing `data/materials.json`. `assets/ui/main_menu.json` controls the menu independently of source code.

## Simulation

- SI-style scale: one world unit is one metre, linear velocities are m/s, angular velocities are rad/s, gravity is m/s², and density is kg/m³.
- Fixed 120 Hz physics with eight solver substeps.
- Cuboid, sphere and capsule colliders; collider and primitive visual dimensions come from the same definitions.
- Density-derived mass properties, static/dynamic friction, restitution, and optional per-definition swept CCD.
- Four restitution solver iterations for multi-point contacts.
- A configurable 0.5 m/s maximum overlap-correction speed limits velocity introduced by numerical penetration correction. This is distinct from impact velocity and does not overwrite a body's momentum.
- Restitution threshold is configured explicitly (1.0 m/s in the sample). Avian treats lower-speed contacts as inelastic to prevent contact jitter; tune this per project if the sample is too dissipative for a chosen material.
- Avian island sleeping is enabled with configurable linear/angular thresholds (0.15 m/s and 0.15 rad/s in the sample) and a 0.5 s rest interval. Bodies are not manually zeroed: resting islands enter the engine's sleeping state, and contact/force changes wake them.
- Contact-start/end counters and a live contact-pair ledger are available to other engine systems.
- Material-specific rolling resistance is applied as a contact impulse only while supported by a contact normal; it does not add global angular damping.
- Camera is a non-physical spectator/editor camera (W/A/S/D, Space/Ctrl, Shift, mouse look). It pauses with the main menu.

The solver settings are external in `data/engine.json`. Restitution belongs to the material definition; it is not a constant baked into source. Avian describes restitution as an approximation: sequential contact solving, friction, speculative collision, and multiple contact points affect the energy response. This is a numerical rigid-body simulation, not a claim of exact continuum-material physics.

## Destructibility

Any cuboid component definition may contain an optional `destructibility` block:

- `impact_speed_threshold_m_s`: minimum approaching contact-normal speed that triggers fracture.
- `fragments_per_axis`: number of pieces along X/Y/Z (each 2–4; no more than 64 total).

The fracture system observes Avian's pre-solver contact-normal speed and contact impulse. When the threshold is crossed, it replaces the original cuboid with a grid of dynamic fragments. Fragment dimensions tile the original collider volume; fragments inherit the original rigid body's linear/angular velocity field. A fractured static block receives the measured contact impulse when its fragments are spawned. Damage and fragment generation are therefore data-driven rather than tied to object names.

A component may also define `bond_strength` with tensile, shear, bending and torsional limits in pascals. Adjacent authored voxels with bond definitions are connected by fixed joints. Joint force and torque are converted to nominal face stresses; a bond is removed when any configured mode exceeds its threshold. This is a beam/section approximation for voxel-scale connections, not a finite-element or continuum stress solver.

Current constraints are deliberate and validated: impact-fracture targets must use cuboid colliders and procedural primitive visuals. Imported glTF fracture geometry is not yet implemented. Fragment pieces remain rigid bodies. The bond-stress calculation is an approximate rigid-body joint failure model, not a deformable continuum simulation.

## Voxel world format

World data is a hierarchy rather than a monolithic world JSON:

1. `world.json` selects one or more region manifests.
2. A region manifest indexes chunk IDs, coordinates and relative chunk paths.
3. Each chunk declares dimensions, voxel size, a block-definition palette, and sparse Y layers. Each row stores one hexadecimal palette index per X cell; rows are ordered along Z. Palette index `0` is air.
4. Non-air palette entries reference cuboid component definitions in `data/components.json`; voxel dimensions scale from the referenced collider dimensions using the chunk's `voxel_size_m`.

The sample overworld contains four 16×16×16 chunks. The chunk coordinate, dimensions and voxel size determine its world-space origin. Each occupied voxel receives a stable ID based on chunk ID and cell coordinates, so runtime transforms, fracture state and saves can refer to a block independently of its neighbours.

Startup reads region indexes and spatial metadata only. Chunk payloads are read asynchronously when the spectator camera approaches, with configured limits for parallel file reads and per-frame chunk spawns. A wider unload radius provides hysteresis. Before unload, dynamic voxel transforms/velocities and detached-fragment state are retained in the world-state resource; broken bonds and fractures remain journaled.

Static, non-fracturable cells are greedily merged into exposed-face render meshes and material-specific compound colliders. Dynamic, destructible, bonded and imported-model cells remain independent rigid bodies. Bond reconnection uses stable voxel identities across chunk boundaries, so loading order does not create duplicate joints. Chunk editing/transactional authoring and seam-aware cross-chunk face merging remain future work.

## Menu and controls

The main menu is loaded from `assets/ui/main_menu.json`. It configures title/subtitle, colors, dimensions, button labels, and actions. Supported actions are `start`, `resume`, and `quit`; the UI system remains in `src/ui.rs`, while all presentation values are external. Press **Escape** to toggle the menu; simulation time and spectator-camera input pause while it is visible.

| Input | Action |
| --- | --- |
| W / A / S / D | Fly forward / left / backward / right |
| Space / Left Ctrl | Fly up / down |
| Left Shift | Sprint |
| Mouse | Look |
| Escape | Open/close main menu |
| F5 | Save rigid-body state, fracture IDs, broken bonds and fragment state |

The camera is intentionally non-physical. It is not a player character and does not participate in collisions.

## Authoring

- `data/engine.json`: physics solver, sleeping, camera and rendering configuration. SI units are encoded in field names.
- `data/materials.json`: density in kg/m³ and dimensionless friction/restitution coefficients.
- `data/components.json`: reusable body, collider, material, swept-CCD, destructibility and visual definitions.
- `worlds/<world>/world.json`: versioned world metadata, region references and optional gravity override.
- `worlds/<world>/regions/*.json`: region indexes.
- `worlds/<world>/regions/chunks/*.json`: chunk voxel data.
- `assets/ui/main_menu.json`: menu presentation and supported actions.
- `assets/`: model and texture paths referenced by `visual.model` are relative to this directory. glTF scene selectors such as `models/part.glb#Scene0` are supported.

Example CLI:

```powershell
cargo run --locked
cargo run -- --world worlds/lab/world.json
cargo run -- --root . --data data --assets assets --world worlds/sandbox/world.json
```

Paths may be absolute or relative to `--root`. The executable resolves its portable root from the directory beside the executable when that directory contains `data/`; development runs fall back to the current working directory.

F5 writes `state.json` atomically and keeps backup/recovery generations. It stores dynamic object transforms and velocities, fractured instance IDs, and live fragment transforms/velocities. Authored static transforms remain in manifests/chunks. Unresolved object records are retained to support content that is temporarily unavailable.

## Local validation

No GitHub-hosted CI workflow is enabled. Run the repository checks on the local development machine with:

```powershell
./scripts/validate.ps1
```

The script checks formatting, type-checks every target, runs all tests and runs Clippy with warnings denied. To omit the lint pass:

```powershell
./scripts/validate.ps1 -SkipClippy
```

The equivalent individual commands are:

```powershell
cargo fmt --all -- --check
cargo check --locked --all-targets
cargo test --locked --all-targets
cargo clippy --locked --all-targets -- -D warnings
```

The committed `Cargo.lock` pins the dependency graph. Keep it under version control and update it deliberately.

A Windows portable release can be assembled with:

```powershell
./scripts/package.ps1
```

The package script copies the executable and complete `data/`, `assets/`, `mods/`, and `worlds/` trees.

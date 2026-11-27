# Human entity data

This directory is the external data package for the `human` entity. Its model files live under `assets/human/`; the eight supplied action JSON files remain in `actions/` without being converted into keyframe animation assets.

## Motion semantics

Each action contains synchronized, timestamped 4×4 transform tracks for the deformation-skeleton bone names. A track is a body-motion target over time. It is not a rendered animation clip and must not directly overwrite the visible skeleton while bypassing rigid-body simulation.

The intended controller samples the action tracks, converts their source coordinate basis, derives the corresponding body/joint target poses and root target, and applies bounded physical actuation to dynamic ragdoll bodies. Contacts, friction, joint constraints, gravity and collisions remain authoritative for the resulting motion. Horizontal root displacement is the result of the physical body interacting with the environment; it must not be fabricated from a constant game-speed translation.

The source uses a Z-up world basis and +Y facing direction. Matrices are stored as row-major 4×4 values and timestamps are in seconds. The action tracks should be sampled by time and interpolated; the source JSON should remain the source of truth.

## Action map

| Gameplay state | Source file |
| --- | --- |
| Idle | `actions/idle.json` |
| Walk | `actions/walk.json` |
| Run | `actions/run.json` |
| Crouch locomotion | `actions/sneak.json` |
| Crouched idle | `actions/kneel.json` |
| Jump | `actions/jump.json` |
| Wall climb | `actions/climb.json` |
| Falling | `actions/fall.json` |

The controller reverses the sampled time direction for backwards walking and crouch locomotion. Running intentionally has no backwards binding. A jump request while facing a nearby wall selects the climb action; otherwise it selects jump. Falling selects the fall action while airborne and descending.

## Temporary test assists

The initial values in `human.json` enable a configurable jump-launch impulse and bounded rotational assistance for testing. Both are deliberately isolated under `temporary_assists` so they can be disabled without changing the action data or ragdoll definition. They are not part of the intended final physical locomotion model.

## Camera and controls

The requested input map and trailing over-the-shoulder camera are data-described in `human.json`. The camera follows the dynamic root, sits slightly above and behind the character, uses a wide perspective field of view, and displays one small center dot rather than a reticle or multi-line crosshair.


## Authored root motion measurement

The supplied Root tracks do not encode meaningful horizontal displacement. The maximum horizontal Root variation across the eight supplied actions is approximately 2 mm (jump); walk, run, crouch and climb have zero horizontal Root displacement in the source data. Locomotion must emerge through physical limb actuation and contact with the ground or wall, not a fixed root velocity.


## Physics scheduling

Input sampling selects the action and publishes contact intent. Jump launch impulses are consumed once, and wall-grip forces are reapplied, by the fixed-step Avian controller. Per-body position and orientation targets are bounded force/torque controllers; the controller never writes horizontal root translation or velocity. The skinned GLB is updated from the resulting rigid-body transforms, while helper bones follow calibrated action targets relative to the physical root.

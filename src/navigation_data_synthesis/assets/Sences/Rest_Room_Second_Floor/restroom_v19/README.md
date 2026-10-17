# Restroom V19 — PhysX startup stabilization

Open `restroom_v19.usda` with its adjacent `layers/` and `assets/` directories intact. For the PhysX configuration and stable GUI startup, run `./start_stable.sh` from this directory. V18 remains unchanged.

## Changes

- Disabled collision on six enclosed toilet-seat hinge-ear meshes that overlapped the fixed control-panel mounts at rest. The visible hinge hardware remains.
- Disabled duplicate collision on the Toilet_03 lid visual meshes. Lid shells, seat rings, support bumpers, doors, frames, wheels, and other functional collision proxies remain enabled.
- Set the default viewport render mode to `RayTracedLighting` and reduced the path-tracing bounce limits from four to two. The high-quality mirror material itself remains available.
- Retained PhysX TGS at 120 Hz and the existing joint limits, masses, inertia, and contact materials. Six concave ceramic fixture LOD colliders remain because their cavities matter for interaction.

## Validation

- `diagnostics/toilet_startup/baseline.json`: 600 PhysX frames. Maximum lid-angle change: 0° for all three lids. Maximum seat-angle change: 0.018°.
- `diagnostics/joint_motion_v19.json`: all five doors and all six toilet seat/lid joints completed open/close phases; maximum target error: 1.04°.
- `comprehensive_audit.json`: 78 top-level objects, 36 rigid bodies (28 enabled), 19 joints, 420 enabled colliders; no reported coordinate, joint-frame, material-binding, or semantic-class errors.
- `movable_collision_coverage_validation.json`: 212 enabled colliders on joint-driven bodies; all checked bodies retain collision coverage.
- `package_validation.json`: no missing asset, external layer dependency, or USD composition error.

These checks validate physics in a separate headless Isaac Sim 6.0.1 PhysX process. Interactive viewport FPS and mirror appearance were not measured. Room dimensions are photo-based estimates, not survey measurements.

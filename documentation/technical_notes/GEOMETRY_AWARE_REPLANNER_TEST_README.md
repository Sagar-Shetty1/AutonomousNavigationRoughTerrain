# Geometry-aware replanner — non-destructive test

This bundle contains a **separate test variant** of the current runtime replanner and a test harness. It does not overwrite `onlineReplanFromPose.m` and does not modify any `.slx` model.

## Requirements

Run `startRoughTerrainProject`, then run the current 45-second simulation so the base workspace contains `scenarioConfig`, `plannerFootprint`, `vehicleClearanceRadius`, terrain arrays, and `simOut` with the replanning logs.

## Run

Place these MATLAB files in the project root:

- `onlineReplanFromPoseGeometryAware.m`
- `testGeometryAwareReplanning.m`

Then call:

```matlab
testGeometryAwareReplanning
```

The test computes an initial route around the two static, configured boxes and a second route from the logged replan start at/after the dynamic obstacle spawn time, avoiding all three boxes. It uses a conservative axis-aligned envelope expanded by the rover footprint radius plus 0.10 m. For any obstacle center not found in `scenarioConfig`, it falls back to the existing circular `inflatedRadius` behavior. Rotated boxes are rejected explicitly.

The test creates a diagnostic figure and prints route and validation results. It does not modify the Simulink model or replace the live `onlineReplanFromPose.m` function.

The initial and post-spawn route lengths begin at different points and therefore should not be compared as if they were the same route.

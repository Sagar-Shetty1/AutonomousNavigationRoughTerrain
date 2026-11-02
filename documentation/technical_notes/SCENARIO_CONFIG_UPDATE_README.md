# Scenario configuration update (Phase 3)

This bundle introduces `defaultRoverScenario.m` as the single source of truth for the current baseline scenario and updates the startup script and MATLAB viewer to consume it.

## Files

- `defaultRoverScenario.m` — defines start/goal, the conservative planning clearance, max waypoints, and three obstacle definitions (shape, XY center, dimensions, static/dynamic state, spawn time).
- `startRoughTerrainProject.m` — calls the scenario function and derives legacy workspace variables required by the existing Simulink Constant blocks and planner. The model is loaded/updated but not simulated or saved.
- `launchElevationTerrainRoverViewer.m` — uses scenario obstacle centers and dimensions and offsets labels to avoid overlap. Terrain is transposed back from the Simscape Grid Surface matrix convention for MATLAB `surf` and `interp2`.

## Apply

1. Back up the current versions of `startRoughTerrainProject.m` and `launchElevationTerrainRoverViewer.m`.
2. Copy these three `.m` files into the project root.
3. In a fresh MATLAB session, run `startRoughTerrainProject`.
4. Confirm it reports `Model updated successfully. Simulation has NOT been started.`
5. Run `simOut = sim('OffRoadMultibodyStage1_OnlineReplanCheckpoint','StopTime','45');`
6. Run `launchElevationTerrainRoverViewer` and inspect the obstacle markers and trajectory.

## Current scope and limitations

- The present physical Simscape model has a single hard-coded 1 x 1 x 0.5 m brick at obstacle 1. Startup asserts that obstacle 1 in the scenario config matches those dimensions.
- Obstacles 2 and 3 are currently configured for planner/sensor logic and visualization; they are not yet all separate physical collision solids in Simscape Multibody.
- The viewer renders box obstacles using each obstacle's configured `[length width height]`. The shape field is recorded in configuration, but shape-aware drawing beyond boxes is not implemented yet.
- The planner still uses its existing 3.5 m circular clearance radius around obstacle centers. It does not yet plan against each obstacle's actual rectangular footprint/dimensions.
- Dynamic obstacle timing in the viewer is based on `scenarioConfig.obstacles(k).spawnTime`; it does not alone prove collision or detection behavior in the physics model.

No `.slx` model files or saved result files are changed by this bundle. This code has not been executed inside MATLAB R2026a by the author; run the startup validation first and report any error before proceeding.

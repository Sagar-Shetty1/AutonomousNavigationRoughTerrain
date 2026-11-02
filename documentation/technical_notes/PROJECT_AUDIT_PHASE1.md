# Autonomous Navigation for Vehicles in Rough Terrain — Phase 1 Audit

**Audit status:** Source and dependency review only. No original project files were edited, renamed, or deleted.

## 1. Executive assessment

The project has working pieces for terrain-aware A* planning, Simscape Multibody rover simulation, online obstacle detection/replanning, and an elevation-colored MATLAB playback viewer. However, these are currently connected through a mixture of separate scripts, old model variants, and MATLAB base-workspace variables. The project is not yet a reproducible, single-entry-point workflow for a new GitHub user.

The immediate goal should be dependency consolidation and scenario consistency—not file deletion or new UI development.

## 2. Inventory summary

From `PROJECT_INVENTORY.txt`:

- **682 files total**
- **453 files under `slprj/`**, classified as generated build/cache artifacts
- **173 files under `resources/`**, mostly project metadata XML
- **56 root-level files**, including models, scripts, CAD assets, experiment results, and recordings
- **11 `.slx` models** and **11 `.slxc` cache files**
- **9 `.m` scripts**
- **5 STL assets**

The 453 `slprj` files and 11 `.slxc` files are strong candidates to exclude from Git. They should not be locally deleted until a clean, reproducible run has been demonstrated. The 173 `resources/project` files should be preserved until the MATLAB Project metadata relationship is understood.

## 3. Core file roles and current dependencies

### `setupOffRoadProject.m`

Builds a 50 m × 50 m procedural terrain, terrain slope/cost arrays, a navigation graph, terrain-only and obstacle-aware A* paths, and a footprint-aware planner. It also creates `X_mb`, `Y_mb`, `Z_mb` for Simscape Multibody.

Important mismatch: it loads and updates `OffRoadTerrainNavigation.slx` and `OffRoadMultibodyStage1.slx`, not the current `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx`. It sets the default start/goal near `(2,2)` and `(48,48)`, whereas the recent online-replanning scenario uses goal `(14,12)`. It does not define all variables referenced by the current checkpoint, including `obstacleXs`, `obstacleYs`, `obstacle3X`, `obstacle3Y`, `obstacleSpawnTime`, `obstacleCenterZ`, `originalPathPadded`, `goalX`, and `goalY` as a complete scenario.

The setup's initial obstacle map also contains three hard-coded rectangular regions at locations different from the configurable `obstacleXs`/`obstacleYs` scenario. These are distinct obstacle representations and should be unified deliberately, not silently merged.

### `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx`

This is the strongest candidate for the current online-replanning baseline because it was used for the recent successful 45-second simulation and contains:

- `DynamicObstacleEnvironment`
- `ObstacleSensorLogic`
- `OnlineReplanManager`
- `GoalStopLogic`
- `PathWaypoints` driven by `originalPathPadded`
- `goalX` and `goalY` constants
- output logs for rover XY position, sensor-detected state, replan event, and replan-start XY

It currently relies on base-workspace values and a MATLAB function `onlineReplanFromPose.m`, which accesses `plannerFootprint` via `evalin('base',...)`. Opening/updating the model in a clean MATLAB session therefore requires an initialization sequence that is not yet represented by one reliable entry-point script.

**Machine-specific asset paths:** the model embeds an absolute path to `C:\Users\sagar\OneDrive\Documents\AutonomousNavigationRoughTerrain\Buggy chassis.STL` and absolute paths to wheel STL files. These must be made project-relative or initialized portably before publishing for other users.

The four wheel File Solid blocks all refer to `RR_Wheel.STL` in the reviewed model. This may be intentional if the meshes are equivalent, but should be verified against the four wheel assets before calling it correct.

### Physical vs. virtual obstacles

The reviewed checkpoint has one `PhysicalObstacle1` Brick Solid with dimensions `[1 1 0.5]` m and one connected Spatial Contact Force for physical contact. `Obstacle1Placement` positions this one physical solid at `obstacleXs(1), obstacleYs(1), obstacleCenterZ`.

Separately, `DynamicObstacleEnvironment` provides two known obstacle centers and a third center after `obstacleSpawnTime` to the detection/replanning logic. The code can therefore plan around three virtual obstacle centers, but the reviewed model has only one obstacle solid/contact pair. The second static obstacle and the third dynamic obstacle are not represented by equivalent physical collision bodies in this model. The current viewer's three obstacle boxes are graphics only.

### `onlineReplanFromPose.m`

Core function used by `OnlineReplanManager`. It copies `plannerFootprint.Graph`, blocks graph links within the required center-distance clearance, runs A*, pads the path to a fixed waypoint matrix, and checks clearance from each obstacle centre. It requires `plannerFootprint` in the base workspace and is documented for Normal-mode simulation rather than code generation.

The current online planner uses a nominal 3.5 m obstacle-centre clearance, with a hard safety radius of 2.25 m when the start is already inside that larger margin. This clearance policy is centre/radius-based, not yet based on configurable obstacle polygons/rectangles/circles and their dimensions.

### `planAroundConfiguredObstacle.m` and `runMultiObstacleValidation.m`

These form a separate experiment workflow targeting `OffRoadMultibodyStage1_RandomNavigation`. The planner script expects `newPath`, `plannerFootprint`, and configured obstacle arrays; then it writes `PathWaypoints`, `ReplannedPath`, `ObstacleX`, and `ObstacleY` into that specific model. It uses its own 3.5 m exclusion radius.

`runMultiObstacleValidation.m` tests two fixed layouts and simulates the RandomNavigation model. It is not a direct runner for the OnlineReplanCheckpoint model. Its saved results are useful experiment evidence, but not the main user workflow yet.

### `runRandomDestinationTrials.m`

Another separate experiment, targeting `OffRoadMultibodyStage1_SpeedTest_15ms`. It samples destinations and records deviations/goal distances. Preserve it as experiment tooling until model variants have been compared.

### `test.m`

A separate kinematic Pure Pursuit/Ackermann script requiring `car`, `derivative`, and `waypoints`. It is not self-contained and is not part of the Simscape checkpoint pipeline. Candidate for an `experiments/` or legacy archive folder after confirming no current workflow calls it.

### `saveOffRoadResults.m`

Saves the current result structures to `offRoadResults.mat` using the current working directory. It is a small utility, but should eventually use a project-root path and a consistent results schema.

### `launchElevationTerrainRoverViewer.m`

Working separate MATLAB visualization. It reads `X_mb`, `Y_mb`, `Z_mb`; when `simOut` exists, it replays `mb_px` and `mb_py`. It estimates yaw from XY movement unless `roverPoseLog` is provided. The displayed rover is a simplified proxy (not the imported CAD geometry and not full 6-DOF pose). The current obstacle graphics use hard-coded dimensions `1.0 × 1.0 × 0.5` m and are explicitly visualization-only.

Obstacle bottoms are set to the highest terrain sample under their footprint plus 8 mm. This avoids clipping the box into the high points but can make it visibly float over lower parts of uneven terrain. The viewer is not currently connected to actual obstacle geometry poses or the Simulink physical collision state.

## 4. Important terrain coordinate-convention issue

`setupOffRoadProject.m` defines `Z_mb` using a transpose:

```matlab
Z_mb = Z_multibody(1:mbStep:end,1:mbStep:end)';
```

That is consistent with the Simscape Multibody Grid Surface convention: **rows in `ZHeights` correspond to X samples and columns correspond to Y samples**. By contrast, MATLAB `surf(X,Y,Z)` expects rows to correspond to Y and columns to X.

The current viewer passes `Z_mb` directly to `surf` and also uses that same array for `interp2`. Since the present X and Y vectors have the same length, the square size check does not catch the transpose. This can swap terrain features across the X/Y diagonal in the viewer and can put rover ground heights and obstacle bases at the wrong elevation relative to the physical Grid Surface.

Before calling the viewer spatially accurate, create a viewer-format height matrix (normally `ZViewer = Z_mb.'`) and use that consistently for plotting/interpolation. Keep `Z_mb` in the Simscape convention for the block. Validate the mapping using an asymmetric known terrain feature before changing the physical model.

Reference: MathWorks, [Grid Surface — X/Y vector and Z Heights conventions](https://www.mathworks.com/help/sm/ref/gridsurface.html).

## 5. Online-replanning log interpretation from the recent 45-second run

The supplied diagnostic shows three rising edges in `onlineReplanEventLog`, at approximately **0.20 s, 2.78 s, and 8.20 s**. The configured third obstacle spawns at 8.00 s, so the 8.20 s event is consistent with the dynamic obstacle being introduced and then processed after the manager's 10-step, 0.02 s quiet period.

`sensorDetectedLog` is a latched `anyDetected` state: it goes high after the first detection and stays high. It is not a pulse or count of new detections. For detailed event reporting, log `newDetection` and detected obstacle coordinates/timestamps separately. The observed three replan event edges are more informative than the continuously-high `sensorDetectedLog`.

The sensor chart sets `halfFOV = pi`, despite a comment saying “45 degrees on either side”. `pi` radians is a 180-degree half-angle, i.e. it covers the full normalized bearing range rather than a narrow forward cone. Decide whether the intended sensor is forward-facing or omnidirectional, then make the code and comment match.

## 6. Candidate file policy (no deletions yet)

| Group | Examples | Recommendation |
|---|---|---|
| Current simulation baseline | `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx` | Preserve; use as comparison baseline until model-variant audit is finished. |
| Terrain/navigation source | `setupOffRoadProject.m`, `onlineReplanFromPose.m` | Preserve; refactor initialization/configuration rather than deleting. |
| Legacy scenario runners | `planAroundConfiguredObstacle.m`, `runMultiObstacleValidation.m`, `runRandomDestinationTrials.m` | Preserve under experiments until cross-references are checked. |
| CAD assets | `Buggy chassis.STL`, `FL_Wheel.STL`, `FR_Wheel.STL`, `RL_Wheel.STL`, `RR_Wheel.STL` | Preserve; likely required by File Solid blocks. Verify wheel-file hashes and intended block-to-file mapping. |
| Build caches | `slprj/**`, `*.slxc` | Exclude from Git. Remove locally only after clean rebuild/reproduction is proven. |
| Workspace snapshot | `matlab.mat` | Preserve an archive copy, but do not treat it as the canonical startup mechanism. It previously emitted an invalid/deleted `plannerAStar` object warning. |
| Experiment result MAT files | `offRoadResults.mat`, `threeObstacleOnline*.mat`, etc. | Keep as evidence/archive until their useful fields and provenance are documented. Avoid making them required inputs. |
| Model variants/checkpoints | `GoalStopBaseline`, `MultiObstacleCheckpoint`, `RandomNavigation`, `SpeedTest*`, `ColoredTerrain` | Do not delete yet. Compare structures and parameter differences first; then retain a small set of milestone checkpoints. |
| MATLAB project metadata | `resources/project/**`, `.prj` | Preserve until confirmed whether the project file is still used. |
| Backups/autosaves | `.original`, `.asv` | Archive temporarily; exclude from Git. Do not delete until key models have a verified backup. |
| Demonstration output | `.avi`, `.mp4`, PNG plots | Select a small number for `docs/`/`results/`; retain full raw outputs locally or outside source control. |
| Pure Pursuit example artifacts | `FollowWaypointsUsingPurePursuit.slx`, example `.m`, `.prj` | Likely separate reference material; keep outside the final core tree unless needed for comparison. |

## 7. Recommended target architecture

The desired five-stage user flow is feasible if all stages consume one scenario definition rather than hard-coded independent variables:

1. **Scenario configuration:** terrain definition, start and goal, and obstacle array with shape, dimensions, centre/pose, and spawn/motion timing.
2. **Interactive 2D editor:** terrain map used to select start/goal and draw/edit obstacles.
3. **Planner:** consumes the scenario and rover footprint; displays the planned 2D route and reports feasibility/clearances.
4. **Simulation runner:** initializes the selected model through a controlled script/`Simulink.SimulationInput`; passes the same obstacle configuration to physical geometry, sensors, and online replanning.
5. **Playback viewer:** consumes the same scenario plus logged actual pose (at least XYZ + orientation quaternion/Euler) and displays actual obstacles using the same dimensions and spawn times.

A single `scenario` structure should be authoritative for obstacle geometry, planning inflation, physical block dimensions/poses, sensor inputs, and viewer graphics. The viewer must not independently hard-code dimensions or infer obstacle existence separately from the scenario. A dedicated initializer should rebuild terrain/planner objects from source and must not depend on loading the large `matlab.mat` snapshot.

## 8. Next audit step required before file cleanup

The supplied bundle contains only the OnlineReplanCheckpoint model, not the other ten `.slx` model files or the five STL files. Therefore it is not yet possible to establish which model variants are duplicates or which wheel meshes are interchangeable.

Create a second read-only archive containing **all root-level `.slx` and `.STL` files**, but no `.slxc`, `slprj`, or result datasets. Then compare the variants and asset hashes. Only after that comparison should we pick one main model and archive redundant checkpoints.

**No files have been deleted or changed as part of this audit.**

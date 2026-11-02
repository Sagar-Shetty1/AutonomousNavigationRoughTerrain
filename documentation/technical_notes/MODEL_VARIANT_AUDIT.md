# Autonomous Navigation for Vehicles in Rough Terrain
## Phase 2 — Model Variant and Geometry Audit

**Audit date:** 19 October 2026
**Scope:** `ModelVariantReview.zip` containing 11 SLX files and five STL assets.
**Method:** Read-only inspection of the SLX package XML, nested subsystem block hierarchy, embedded Stateflow/MATLAB Function scripts, model configuration, referenced external geometry paths, and STL mesh geometry. No source file was modified.

---

## Executive recommendation

Use **`OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx` as the current technical baseline for continued work**. It is the most feature-complete model in this bundle: it contains the physical rover, online replanning, the time-triggered third obstacle in the sensor/planner environment, event logging, one explicit physical obstacle solid, and a Spatial Contact Force connection for that obstacle.

This is a **provisional development baseline**, not yet the final end-to-end project. The current setup script initializes different, older models and does not create all variables required by the online-replanning checkpoint. That needs to be corrected before the repository can reliably support a new user starting from a clean MATLAB session.

Do not delete any model, asset, MAT file, or cache directory yet. Keep the present files intact while we build a repeatable setup process and verify a clean run.

## 1. Model comparison

The “block entries” counts below are counts of recursively traversed block entries in the saved model hierarchy. They are useful for comparing variants, but are not a Simulink-reported block count.

| Model | Block entries | Model features found | Configuration / disposition |
|---|---:|---|---|
| `FollowWaypointsUsingPurePursuit.slx` | 34 | Standalone waypoint following / pure-pursuit example. No full multibody rover assembly. | Earlier example; retain as reference/archive, not the main project model. |
| `OffRoadTerrainNavigation.slx` | 38 | Earlier kinematic navigation model (Pure Pursuit / Ackermann Kinematic Model). No CAD rover/tire-contact assembly. | Legacy controller example; retain until dependencies are understood. |
| `OffRoadMultibodyStage1.slx` | 249 | Early multibody rover version; controller blocks include generic `MATLAB Function` / `MATLAB Function1`. No later online obstacle-replanning stack. | Historical baseline; retain as a recovery checkpoint. |
| `OffRoadMultibodyStage1_GoalStopBaseline.slx` | 260 | Multibody rover with `GoalStopLogic`; no obstacle sensing, online replanning manager, or separate physical obstacle/contact pair. | Useful fallback for rover and goal-stop behavior. Stop time 30 s, `ode15s`. |
| `OffRoadMultibodyStage1_MultiObstacleCheckpoint.slx` | 283 | Obstacle-sensing subsystem and `ReplannedPath` switch/input exist; no later `DynamicObstacleEnvironment` / `OnlineReplanManager` combination and no separate obstacle solid + contact force. | Earlier multi-obstacle planning experiment; retain as an experiment/checkpoint. Stop time 45 s, `ode15s`. |
| `OffRoadMultibodyStage1_RandomNavigation.slx` | 299 | Obstacle sensing, `OnlineReplanManager`, route switching, and a replan event log. Does not have the time-triggered environment subsystem or the separate physical obstacle/contact geometry used in the online checkpoint. | Useful intermediate for randomized/multi-obstacle planning experiments. Stop time 45 s, `ode15s`. |
| `OffRoadMultibodyStage1_SpeedTest.slx` | 261 | Rover/goal-stop style model without the later obstacle stack. | Historical speed/solver experiment. Stop time 30 s, `ode15s`. |
| `OffRoadMultibodyStage1_SpeedTest_14ms.slx` | 261 | Same traversed block paths and parameters as `SpeedTest` (excluding model configuration differences). | The file name suggests a 14 ms step, but no 0.014 s fixed-step value was found in the saved solver configuration. It uses `ode15s`, stop time 30 s, and `RelTol = 2e-3`. Do not treat the suffix as verified solver behavior. |
| `OffRoadMultibodyStage1_SpeedTest_15ms.slx` | 265 | Similar earlier rover/goal-stop variant; has a different `GoalStopLogic` port configuration and extra hierarchy entries. | Not a simple numerical-step clone based on the saved contents. It uses `ode15s`, stop time 45 s, and `RelTol = 2e-3`. Keep until separately validated. |
| `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx` | 327 | Most complete current variant: online replanning manager, obstacle sensor logic with new-detection coordinates, timed third-obstacle environment, event/start-position logs, one `PhysicalObstacle1` Brick Solid, and a Spatial Contact Force block. | **Recommended current technical baseline.** Stop time 45 s, `ode15s`, `RelTol = 2e-3`. |
| `OffRoadMultibodyStage1_ColoredTerrain.slx` | 327 | Same block hierarchy, block parameters, line/connection topology, configuration settings, and embedded MATLAB Function scripts as `OnlineReplanCheckpoint`. It contains no persistent per-vertex terrain-color implementation. | **Duplicate of the online checkpoint**, despite the filename. Archive or remove only after the baseline workflow is reproducible and a backup exists. |

### Duplicate verification: `ColoredTerrain`

`OffRoadMultibodyStage1_ColoredTerrain.slx` and `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx` were compared across:

- Recursively traversed block paths and block types.
- Saved block parameters, including the relevant `InstanceData`.
- Root-level connection signatures.
- Model solver configuration.
- Embedded MATLAB Function / Stateflow chart names and script contents.

No differences were found in these categories. The names and file metadata differ, but the color experiments did not leave a different model implementation. Keep the duplicate as a recovery copy for now; do not develop two parallel “active” models.

## 2. Physical obstacle implementation and current limitation

The online checkpoint contains:

- `PhysicalObstacle1`, a `Brick Solid` with `BrickDimensions = [1 1 0.5]` m.
- `Obstacle1Placement`, whose translation is `[obstacleXs(1), obstacleYs(1), obstacleCenterZ]`.
- One separate `Spatial Contact Force` block associated with the physical obstacle.
- `ObstacleSensorLogic`, `DynamicObstacleEnvironment`, and `OnlineReplanManager` for the virtual sensing/replanning layer.

The dynamic environment function outputs fixed-length three-entry X/Y arrays. Obstacles 1 and 2 are always supplied from `knownXs`/`knownYs`; obstacle 3 is `NaN` until `t >= spawnTime`, after which its configured X/Y coordinates are supplied. The online-replanning manager has `maxObstacles = 3`, `maxWaypoints = 100`, and waits 10 samples at a 0.02 s sample time before replanning. Its call to `onlineReplanFromPose` passes a nominal inflated radius of 3.5 m.

**Important limitation:** only obstacle 1 has the separate physical solid/contact connection in this model. The additional obstacle coordinates drive sensing/planning logic, but they do not each have independent physical collision solids in the reviewed model. The MATLAB viewer draws its own graphics; that display is not evidence of physical collision geometry.

Also, the current planner API accepts obstacle center X/Y coordinates and one scalar inflated radius. It does not receive each obstacle’s shape or dimensions. Therefore, it does not yet support geometrically accurate planning for arbitrary user-defined circles, rectangles, or differently sized obstacles. That is a key architectural change for the proposed user interaction flow.

### Sensor field-of-view inconsistency

In the `ObstacleSensorLogic` function, the saved code contains:

```matlab
sensorRange = 10.0;
halfFOV = pi;  % 45 degrees on either side
```

`pi` is 180 degrees, not 45 degrees. If the intent is a forward sensor with a 45-degree half-angle, the intended value would be `pi/4`; however, do not change this silently until we confirm the desired sensor behavior and re-run the existing tests. As saved, the condition provides no meaningful forward-only angular restriction.

The sensor’s `detected` output is a latched flag (`anyDetected` stays true after a detection), not a per-obstacle pulse. This explains why `sensorDetectedLog` can remain 1 for the rest of a run and should not be interpreted as repeated detections. For event timing, `onlineReplanEventLog` is the more useful logged signal. In the previously inspected 45-second run, replan event pulses occurred at approximately 0.20 s, 2.78 s, and 8.20 s. The final event is consistent with the configured 8 s dynamic-obstacle appearance, but the model should log `newDetection` and detected obstacle coordinates explicitly to prove which obstacle caused each replan.

## 3. STL asset audit

### Asset references inside the model files

The reviewed multibody variants store absolute geometry paths such as:

- `C:\Users\sagar\OneDrive\Documents\AutonomousNavigationRoughTerrain\Buggy chassis.STL`
- `C:\Users\sagar\OneDrive\Documents\AutonomousNavigationRoughTerrain\RR_Wheel.STL`

All four wheel `File Solid` instances in the saved models reference `RR_Wheel.STL`; the other wheel STL names are not referenced by those model instances. Geometry file units are set to millimetres.

This is a **GitHub portability blocker**: a clone in another directory or on another computer will not resolve these exact absolute paths unless the geometry references are repaired. In a future cleanup, keep a defined `assets/geometry/` directory and use a model/setup strategy that resolves paths from the project root. Preserve the `mm` geometry units.

### Mesh comparison

| Asset | File size | Mesh result |
|---|---:|---|
| `Buggy chassis.STL` | 15,525,284 bytes | 310,504 faces; extents approximately 911.0 × 689.9 × 1426.6 mm. |
| `FL_Wheel.STL` | 1,488,334 bytes | 29,765 faces; extents approximately 52.1 × 152.0 × 152.0 mm. |
| `FR_Wheel.STL` | 1,488,334 bytes | Same triangle geometry as `FL_Wheel.STL`. |
| `RL_Wheel.STL` | 1,488,334 bytes | Same triangle geometry as `FL_Wheel.STL`. |
| `RR_Wheel.STL` | 1,488,334 bytes | Same triangle geometry as `FL_Wheel.STL`. |

The four wheel files have different binary hashes, but their sorted triangle-coordinate sets match exactly in the mesh comparison. That makes `FL_Wheel.STL`, `FR_Wheel.STL`, and `RL_Wheel.STL` **candidate duplicate assets for this Simulink model**, not confirmed files to delete. They may be useful as exports from the original CAD assembly, so archive them first and keep at least one verified wheel mesh required by the model.

The chassis STL is not geometrically evaluated as “correct” or “incorrect” by these bounds alone; its placement and transforms must be checked in the model.

## 4. Setup/workspace inconsistency

`setupOffRoadProject.m` currently initializes:

- A 50 m × 50 m synthetic terrain on a 0.1 m grid.
- A terrain cost map and several A* routes, including a footprint-aware route.
- `start = [2, 2]` and `goal = [48, 48]`.
- Fixed rectangular regions in an `obstacleMask`.
- Downsampled multibody terrain variables `X_mb`, `Y_mb`, `Z_mb`.
- Legacy model names: `OffRoadTerrainNavigation` and `OffRoadMultibodyStage1`.

But the online checkpoint requires variables such as `obstacleXs`, `obstacleYs`, `obstacle3X`, `obstacle3Y`, `obstacleSpawnTime`, `obstacleCenterZ`, `originalPathPadded`, `goalX`, and `goalY`. The setup script does not initialize all of these or load/update the online checkpoint. Its goal is also different from the more recent online-replanning tests, which used goal `(14, 12)`.

The saved workspace `matlab.mat` is not a suitable replacement for setup: it can contain saved planner objects that show an “Invalid or deleted object” warning when loaded. We should rebuild the required route/configuration data from scripts and a plain scenario configuration, rather than asking new users to load a historical workspace image.

## 5. Terrain matrix convention mismatch in the MATLAB viewer

The setup script creates the Simscape terrain with:

```matlab
Z_mb = Z_multibody(1:mbStep:end,1:mbStep:end)';
```

The transpose is intentional for the Simscape Multibody `Grid Surface` convention: the rows of `Z Heights` correspond to the X grid vector, and columns correspond to the Y grid vector. The MathWorks documentation states this explicitly: [Grid Surface documentation](https://www.mathworks.com/help/sm/ref/gridsurface.html).

The viewer currently checks `Z_mb` as though its rows correspond to Y and its columns correspond to X, then passes it directly to MATLAB `surf` and `interp2`. MATLAB meshgrid-style surfaces use a Z matrix whose rows correspond to Y and columns to X. Because this terrain is square (251 × 251), the mistaken orientation passes the viewer’s size check instead of raising an error.

**Required correction before claiming spatially accurate viewer output:** create a visualization-oriented matrix such as `Zview = Z_mb'` for `surf`, `interp2`, rover ground-height interpolation, and obstacle base-height sampling. Do not transpose the model’s `Z_mb` in place or change the Simscape `Grid Surface` parameter without separately validating the physics terrain. Use explicit names such as `Z_simscape` and `Z_meshgrid` so the convention can no longer be confused. Supporting rectangular grids will also make this mismatch detectable.

## 6. Proposed target architecture for the five-stage project

The five desired stages are feasible, but should share one authoritative scenario definition instead of independent hard-coded workspace variables.

### Shared scenario (source of truth)

A future `scenario` configuration should contain at least:

- Terrain grid, axes, units, elevation convention, and any customizable terrain parameters.
- Start pose/XY and goal XY.
- Vehicle footprint and relevant clearance parameters.
- An obstacle list with ID, shape, XY position, dimensions, static/dynamic type, and spawn time (when applicable).
- Planner options, grid resolution, terrain-cost settings, and simulation duration.

The planner, Simulink model setup, and 3D viewer should all consume this same scenario. If a shape or dimension is not supported by the current physical model, the UI must say so rather than draw a shape that differs from the one used for collision checking.

### Execution sequence

1. **2D terrain map:** inspect/configure terrain; click to choose start and goal.
2. **Obstacle editor:** create/delete obstacles and enter shape, position, dimensions, and optional spawn time.
3. **2D planned-path plot:** display the path, terrain costs/slope, start/goal and obstacle boundaries; validate clearance.
4. **Simscape Multibody run:** transfer the same scenario into physical/visual obstacle geometry and collect actual poses/events.
5. **3D viewer:** use the corrected terrain orientation and simulation pose data, render obstacles from the same dimensions and timing, and differentiate planned route from actual traversal.

For the physical obstacle stage, a first feasible version can use a bounded number of supported obstacle instances with box/cylinder geometry, created/configured before simulation. Arbitrary obstacle counts and runtime physical spawn/motion should be designed explicitly; they cannot be achieved merely by adding viewer patches.

## 7. Safe cleanup recommendations

### Preserve now

- `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx` — current technical baseline.
- `setupOffRoadProject.m`, `onlineReplanFromPose.m`, `planAroundConfiguredObstacle.m`, `launchElevationTerrainRoverViewer.m` — current source logic, even though integration needs work.
- At least one fallback model such as `OffRoadMultibodyStage1_GoalStopBaseline.slx`, plus `MultiObstacleCheckpoint` and `RandomNavigation` while their differences are still useful.
- `Buggy chassis.STL` and `RR_Wheel.STL` — confirmed assets used by the current model.
- Existing results/plots/recordings until we decide which are needed for validation and GitHub examples.
- The existing `.prj`/`resources/project` metadata until project startup and dependencies are checked.

### Potential archive candidates (do not delete yet)

- `OffRoadMultibodyStage1_ColoredTerrain.slx` — duplicate of `OnlineReplanCheckpoint`.
- Older speed-test variants after their intended test difference is documented.
- Older controller examples if they are not required by the final workflow.
- `FL_Wheel.STL`, `FR_Wheel.STL`, `RL_Wheel.STL` — duplicate meshes in the current bundle, but preserve provenance until the CAD/export workflow is confirmed.
- `.slxc`, `slprj/` build products, `*.asv`, `.original`, generated videos, and historical MAT results: classify one by one.

For GitHub, `.slxc` and `slprj/` should normally be excluded as regenerable MATLAB/Simulink build products. Do not remove them locally until a fresh initialization and simulation succeeds from the source files. Some result MAT files or demonstration videos may be worth retaining selectively.

---

## Recommended next phase

1. Create a plain scenario configuration and an initialization script that starts the **online checkpoint** from a clean MATLAB session without loading `matlab.mat`.
2. Correct and validate the viewer’s X/Y/Z matrix convention using a non-symmetric landmark/test (for example, a known hill or obstacle at a known XY coordinate).
3. Add explicit logging for actual chassis position, vertical position, and orientation, alongside per-detection/replanning events.
4. Repair absolute STL references so the project works from a fresh clone.
5. Only after a clean-start simulation and viewer test, archive duplicates and generated clutter.

**No source files or assets were deleted or modified during this audit.**

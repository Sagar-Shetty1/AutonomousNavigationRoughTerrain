# Session startup and verification

This bundle contains a safer fresh-session workflow for the current rough-terrain rover project.

## Files

- `setupOffRoadProject.m`: rebuilds the terrain, terrain-cost grid, navigation graph, and base planner. It intentionally no longer loads older Simulink models.
- `startRoughTerrainProject.m`: configures the current three-obstacle test scenario, creates the initial route, computes the vertical placement of the existing physical brick from the terrain, and loads/updates only `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx`.
- `launchElevationTerrainRoverViewer.m`: fixes the coordinate convention by transposing the Simscape-format `Z_mb` back to MATLAB's `surf`/`interp2` convention.

## Apply safely

1. Make backups of the existing `setupOffRoadProject.m` and `launchElevationTerrainRoverViewer.m` in your project folder.
2. Copy these supplied `.m` files into the project root, replacing the originals only after the backups exist.
3. Restart MATLAB (or use a new MATLAB session) and set the project root as the current folder.
4. Run:

   ```matlab
   startRoughTerrainProject
   ```

5. Confirm the script ends with `Model updated successfully. Simulation has NOT been started.`
6. Start the test simulation separately:

   ```matlab
   simOut = sim('OffRoadMultibodyStage1_OnlineReplanCheckpoint', 'StopTime', '45');
   ```

7. Open the viewer:

   ```matlab
   launchElevationTerrainRoverViewer
   ```

## Scenario configured by the entry point

- Start pose: `[2, 2, pi/4]`
- Goal: `(14, 12)` m
- Static obstacle centers: `(9, 6.5)` m and `(11.5, 8)` m
- Dynamic obstacle marker: `(13.5, 5)` m, spawn time `8` s
- Existing physical Brick Solid: `1 x 1 x 0.5` m

**Current limitation:** this startup step does not add physical collision geometry for all three obstacles. The current Simulink model includes one physical brick while the viewer can show all configured obstacles. This must be addressed as a later implementation task.

## What this workflow does not do

- It does not delete, rename, or modify any `.slx` file on disk.
- It does not start the simulation automatically.
- It does not guarantee the 45-second simulation will complete; any solver error should be diagnosed from its actual report rather than by changing solver settings pre-emptively.

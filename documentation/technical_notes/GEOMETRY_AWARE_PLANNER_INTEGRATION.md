# Geometry-aware online replanner integration

This bundle contains the geometry-aware planner that passed `testGeometryAwareReplanning`, adapted to the existing live function name `onlineReplanFromPose`. It keeps the existing six-input/four-output function interface expected by the MATLAB Function block and does not modify the `.slx` model.

## What changes

- Configured, axis-aligned box obstacles are converted into conservative axis-aligned envelopes.
- Envelope half-extents are `obstacle dimension / 2 + vehicleClearanceRadius + geometryMargin`.
- The planner blocks graph links intersecting each inflated box and validates returned path segments against the same envelope.
- Unknown obstacle centers and unsupported shapes retain the legacy circular fallback.
- Rotated boxes are rejected explicitly rather than silently treated as axis aligned.

The current scenario uses axis-aligned 1 m x 1 m x 0.5 m boxes and a 0.10 m additional planning margin. This remains a *conservative 2-D planner model*, not exact contact geometry. The Simscape model still has only one explicit physical obstacle solid; this bundle does not add physical obstacles 2 or 3.

## Safe installation

1. Extract this ZIP into a **temporary folder outside the project root** (for example, a new `GeometryAwarePlannerIntegration` folder under Downloads). Do not extract/overwrite directly into the project root yet. Close any running simulation; keep current model and result files untouched.
2. In MATLAB, confirm the Current Folder is the project root, then create a `backups` directory if needed and make a plain-text backup of the live planner and scenario configuration:

   ```matlab
   projectDir = pwd;
   backupDir = fullfile(projectDir, 'backups');
   if ~isfolder(backupDir), mkdir(backupDir); end
   copyfile(fullfile(projectDir,'onlineReplanFromPose.m'), ...
       fullfile(backupDir,'onlineReplanFromPose_preGeometryAware.txt'));
   copyfile(fullfile(projectDir,'defaultRoverScenario.m'), ...
       fullfile(backupDir,'defaultRoverScenario_preGeometryAware.txt'));
   ```

3. Copy the extracted bundle's `onlineReplanFromPose.m` and `defaultRoverScenario.m` into the project root, replacing the corresponding files only after the backups above exist.
4. Clear the cached function and rebuild the startup workspace:

   ```matlab
   clear onlineReplanFromPose
   rehash
   startRoughTerrainProject
   ```

   Startup should print the initial box-aware replanning diagnostics and successfully update the model. It must not start the simulation.
5. Run a limited integration check through the 8.2 s dynamic-obstacle replan first:

   ```matlab
   simOutGeometryCheck = sim( ...
       'OffRoadMultibodyStage1_OnlineReplanCheckpoint', ...
       'StopTime', '12');
   ```

   Confirm that replanning occurs around 0.2 s, 2.78 s and 8.2 s and that the route function reports successful envelope validation. If the simulation errors or the rover trajectory changes unexpectedly, stop here and restore `onlineReplanFromPose.m` from the text backup before proceeding.

6. Only after the 12 s check passes should you run the full 45 s simulation and inspect the viewer.

## Rollback

Restore the source from `backups/onlineReplanFromPose_preGeometryAware.txt` to the project-root file `onlineReplanFromPose.m`, then run:

```matlab
clear onlineReplanFromPose
rehash
startRoughTerrainProject
```

No Simulink model files are modified by this bundle or by the startup script.

# Rough-Terrain Scenario Editor + Physical Workflow: Multi-Obstacle Update

## What changed

This update removes the previous hard stop when the scenario editor contains an extra obstacle such as Obstacle 4.

- Supports 3–10 configured axis-aligned box obstacles, with unique positive-integer IDs.
- Keeps the calibrated core roles: ID 1 is static; IDs 2 and 3 are dynamic and must retain the already validated centers and dimensions required by the world-Z spawn calibration.
- Extends the physical test model's virtual obstacle environment and online replanning capacity to 10 slots.
- Preserves the 18 m sensor range and the current world-Z calibration for physical obstacles 2 and 3.
- Obstacles with IDs 4–10 participate in virtual sensing, planning, and viewer display. They do **not** get separate Simscape collision bodies in this version.
- The original `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx` and `OffRoadMultibodyStage1_ScenarioIntegrationTest.slx` are not edited by this runner. The isolated physical v2 test model is backed up before synchronization.

## Important: preserve the current unsaved editor scenario

The screenshot showed the physical workflow validation dialog while Obstacle 4 was selected. In the previous editor version, validation occurs *before* the Run action saves `scenarioConfig.mat`.

1. Click **OK** to dismiss the current error dialog.
2. In the still-open editor, click **Save scenario MAT**. Confirm it reports the project-root `scenarioConfig.mat` path. This preserves the fourth obstacle and the current start/goal values.
3. Close the old editor window before replacing its script.

## Install

Copy these two `.m` files into the MATLAB project root, replacing the same-named scripts only after backing them up:

- `launchRoughTerrainScenarioEditor.m`
- `runPhysicalScenarioWorkflow.m`

Project root:
`C:\Users\sagar\OneDrive\Documents\AutonomousNavigationRoughTerrain`

## Run

In MATLAB, from the project root:

```matlab
clear launchRoughTerrainScenarioEditor runPhysicalScenarioWorkflow
rehash
launchRoughTerrainScenarioEditor
```

Your saved `scenarioConfig.mat` will load in the editor. Verify obstacle IDs 1–4 and the saved positions/dimensions. Click **Save + Run Physical Workflow**.

The runner will use the isolated `OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_v2.slx`, widen its virtual environment and manager to 10 obstacle slots, synchronize the saved scenario, run the configured stop time, check logs, and open the multi-dynamic viewer.

## Scope and caveats

- Maximum: 10 box obstacles. All IDs must be unique positive integers; IDs 1, 2, and 3 must be present.
- IDs 1–3 remain the calibrated physical core. Obstacle 1 is the existing physical static brick; obstacles 2 and 3 are the calibrated physical-spawn bodies.
- IDs 4–10 are virtual only: no corresponding physical collision solids are created.
- The obstacle 2/3 Z=0 spawning is a calibrated guided joint-stop approximation, not true terrain contact. Because the box centers target world Z=0, boxes may intersect higher terrain.
- The physical calibration guard remains in place. If obstacle 2 or 3 is moved or resized, physical execution stops until its spawn calibration is revalidated.
- This update was source-checked in the artifact environment but not executed inside the user's MATLAB/Simulink installation. Validate the first run before relying on it.

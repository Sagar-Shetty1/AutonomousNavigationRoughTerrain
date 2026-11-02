# Physical Rough-Terrain Workflow (v2)

This bundle creates a distinct physical-spawn model and a one-command runner.
It does not overwrite `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx`,
`OffRoadMultibodyStage1_ScenarioIntegrationTest.slx`, or the earlier physical test model.

## Files
- `createPhysicalScenarioSpawnTestModel_v2.m`: builds `OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_v2.slx` from the working ScenarioIntegrationTest model, adds physical bodies for obstacles 2/3, sets sensor range to 18 m, and applies the calibrated world-Z spawn/joint settings. It does not simulate.
- `runPhysicalScenarioWorkflow.m`: restores workspace dependencies and `scenarioConfig.mat`, creates the v2 model if missing, synchronizes constants/start pose and calibrated physical settings, runs the configured simulation, reports replanning events, and opens the multi-dynamic viewer.

## Install
Copy both `.m` files to:
`C:\Users\sagar\OneDrive\Documents\AutonomousNavigationRoughTerrain`

Ensure the project already contains:
- `scenarioConfig.mat` (save from the Scenario Editor first)
- `setupOffRoadProject.m`
- `OffRoadMultibodyStage1_ScenarioIntegrationTest.slx`
- `onlineReplanFromPoseGeometryAware.m`
- `launchElevationTerrainRoverViewerMultiDynamic.m`

If `OffRoadMultibodyStage1_ScenarioIntegrationTest.slx` does not exist yet, run the already working `runRoughTerrainWorkflow` once first. The physical runner does not create or modify that source integration model.

Run this one command:
```matlab
runPhysicalScenarioWorkflow
```

## Calibration and physical limitations
The current empirical world-Z calibration is guarded to these configurations:
- obstacle 2 center `(23.7161198288, 22.8601997147)`, dimensions `[2 23 1]`
- obstacle 3 center `(16.7974322397, 36.0556348074)`, dimensions `[4 4 0.5]`

If those centers or dimensions change, the runner intentionally errors instead of applying an unverified calibration. Recalibration is required. Dynamic spawn times may be updated from the saved scenario, but changing them may alter physical interactions and should be tested.

Obstacles 2/3 are physically present in the model from t=0 and placed mostly below world Z=0 until released via a joint-mode transition. Their centers are empirically calibrated to settle near world Z=0 in the current scenario. The boxes can intersect elevated terrain; the lower/upper joint stop is not terrain-contact geometry. Masses are test placeholders (25 kg and 10 kg), not material-derived masses. The 18 m sensor range was observed to permit a full 45 s test run in the current configuration, but it is an experimentally selected parameter, not a universally optimal sensor range.

These new v2 scripts have not been executed in the user's MATLAB session. The existing v1 physical test model passed 45 s with sensor range 18 m, but the v2 builder/runner must be validated on the user's MATLAB installation before considering the rebuild reproducible.

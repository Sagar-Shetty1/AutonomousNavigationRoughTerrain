# Physical Obstacle 3 Spawn Test

This is a non-destructive test script for a *new model copy* that includes physical obstacle 1, physical obstacle 2, and a third physical box with a prescribed vertical spawn motion. It does not edit `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx` or `OffRoadMultibodyStage1_PhysicalObstacle2Test.slx`.

## Before running

1. Run `startRoughTerrainProject`.
2. Confirm `OffRoadMultibodyStage1_PhysicalObstacle2Test.slx` exists and was created/updated by `createObstacle2PhysicalTestModel.m`.
3. Place `createObstacle3PhysicalSpawnTestModel.m` in the project root.
4. Ensure `OffRoadMultibodyStage1_PhysicalObstacle3SpawnTest.slx` does not already exist. The script will not overwrite it.

## Create the test model

Run:

```matlab
createObstacle3PhysicalSpawnTestModel
```

The script calculates obstacle 2 and 3 center heights from the highest sampled terrain elevation under each footprint, plus 5 mm and half the obstacle height. It then clones the obstacle-2 test model and adds obstacle 3's Brick Solid, a dedicated Prismatic Joint, a Spatial Contact Force connected to the chassis, a Clock/MATLAB Function motion profile, and a Simulink-PS Converter.

The joint uses prescribed motion with a quintic profile and matching first/second derivatives. Obstacle 3 starts 2 m above its final terrain-relative center, begins descending `dropDuration` seconds before the configured spawn time, and reaches the configured center at the spawn time. The current default descent duration is 0.5 s.

## Test only after model update succeeds

```matlab
simOutObstacle3 = sim( ...
    'OffRoadMultibodyStage1_PhysicalObstacle3SpawnTest', ...
    'StopTime', '12');
```

Inspect the model around 7.5–8.0 s in Multibody Explorer and verify the geometry-aware planner's third replan near 8.2 s. If model update fails, do not run the simulation; save the full error message and close any partial test model without saving.

## Limitation

This is a kinematically prescribed spawn proxy, not a freely falling rigid body. In Multibody Explorer, the brick exists 2 m above the ground before it descends, so it may be visible above the terrain before the configured spawn time. The terrain is not currently an obstacle-contact surface; the obstacle contact blocks couple the obstacle geometry to the rover chassis. This test verifies consistent physical box geometry and timed placement, not free-body gravity, terrain impact, or successful collision avoidance by itself.

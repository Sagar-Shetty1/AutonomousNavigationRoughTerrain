# Physical Obstacle 2 Test

This is a non-destructive test script for adding the second static obstacle to a **separate test copy** of the current Simscape Multibody model.

## Run

1. Place `createObstacle2PhysicalTestModel.m` in the project root.
2. Run `startRoughTerrainProject` first, so `scenarioConfig`, `X`, `Y`, `Z_multibody`, `obstacleXs`, and `obstacleYs` are available.
3. Ensure `OffRoadMultibodyStage1_PhysicalObstacle2Test.slx` does not already exist. The script refuses to overwrite it.
4. Run `createObstacle2PhysicalTestModel`.
5. If it reports that the copy updated successfully, run the suggested 12-second simulation command.

The script adds a second 1 x 1 x 0.5 m (by current scenario) Brick Solid, places it above the highest sampled terrain point under its footprint plus 5 mm, and adds a separate Spatial Contact Force connection from the chassis geometry to that obstacle geometry.

The source model `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx` is not edited or saved. If construction or model update fails, the unsaved test copy is closed without saving. The script does not yet add physical obstacle 3 or implement dynamic activation; those remain separate steps.

# Geometry-aware replanning safety-guard fix

## Why this fix is needed

The 12-second integration test stopped during the replanning event with the measured rover pose near `(9.120, 5.008) m` and obstacle 1 centered at `(9.000, 6.500) m`.

The integrated function still applied the legacy `2.25 m` circular start guard to every obstacle. That circle rejected the start because its distance to obstacle 1's center is about `1.497 m`, even though the pose lies outside that configured box's conservatively inflated axis-aligned envelope (`half-extents = 1.243 m`). The guard therefore contradicted the geometry-aware collision envelope used by the planner.

## Change

The patched `onlineReplanFromPose.m`:

- uses the same inflated AABB to guard configured axis-aligned boxes;
- checks both the measured rover pose and the snapped graph state;
- preserves the old `2.25 m` radial guard for unknown or non-box obstacles;
- still raises an error if the rover's measured pose or snapped state is actually inside a box's inflated envelope;
- does not change the Simulink model or solver settings.

This patch supports axis-aligned boxes only. Rotated boxes are still rejected by the function. A successful route test is not, by itself, proof of physical collision avoidance.

## Apply safely

1. Before replacing anything, back up the current geometry-aware `onlineReplanFromPose.m` to `backups/onlineReplanFromPose_geometryAware_preGuardFix.txt`.
2. Copy the provided `onlineReplanFromPose.m` into the project root, replacing the current function.
3. In MATLAB run:

   ```matlab
   clear onlineReplanFromPose
   rehash
   which onlineReplanFromPose -all
   startRoughTerrainProject
   ```

4. Confirm `which` resolves to the project-root function, and startup ends with `Model updated successfully. Simulation has NOT been started.`
5. Run the shortened integration test:

   ```matlab
   simOut12 = sim('OffRoadMultibodyStage1_OnlineReplanCheckpoint', ...
                  'StopTime', '12');
   ```

6. If it completes, inspect the event times:

   ```matlab
   ev = simOut12.get('onlineReplanEventLog');
   t = ev.Time(:);
   d = squeeze(ev.Data); d = d(:);
   disp(t(find(d ~= 0)));
   ```

Expected event times should be close to the existing trigger times (approximately 0.2 s, 2.78 s and 8.2 s), but timing and route feasibility must be confirmed from the actual run. If the simulation errors again, keep the full error and do not run the 45-second simulation yet.

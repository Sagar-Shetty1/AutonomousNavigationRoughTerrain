# Autonomous Navigation for Vehicles in Rough Terrain

A MATLAB and Simulink-based autonomous rover navigation project that combines terrain-aware path planning, vehicle-footprint obstacle avoidance, online obstacle detection, dynamic replanning, and Ackermann path following in a Simscape Multibody environment.

## Project Overview

The project develops an autonomous rover capable of navigating a rough, uneven terrain environment while avoiding static and dynamically appearing obstacles.

The navigation system combines:

- Terrain generation and elevation-based cost mapping.
- A* graph-based path planning.
- Vehicle-footprint-aware obstacle avoidance.
- Obstacle detection and online route replanning.
- Pure Pursuit path following.
- Ackermann steering geometry.
- Simscape Multibody rover simulation.
- Elevation-coloured terrain and obstacle visualization.

## System Architecture

### 1. Terrain Modelling

A 50 m × 50 m terrain environment is generated in MATLAB.

The terrain includes hills, ridges, depressions, and surface roughness. Terrain gradients are calculated to estimate slope, and terrain traversal costs are assigned according to slope severity.

### 2. Path Planning

The navigation planner uses an 8-connected graph and A* search.

Three planning approaches are implemented:

- Terrain-aware planning.
- Obstacle-aware planning.
- Vehicle-footprint-aware planning.

The geometry-aware replanner accounts for configured obstacle dimensions and rover clearance when generating replacement routes.

### 3. Obstacle Detection and Online Replanning

The rover uses a simulated forward obstacle-sensing system.

When a previously unknown obstacle is detected, its position is added to the known obstacle set. The online replanning manager then generates a replacement route from the rover's current position.

The demonstration scenario contains six obstacles:

- Static obstacles: O1, O2, O4 and O6.
- Dynamic obstacles: O3 and O5.
- O3 activation: 7 seconds.
- O5 activation: 15 seconds.

The three original physical obstacles remain in the Simscape model. The complete six-obstacle scenario is represented in the visualization layer.

### 4. Path Following and Steering

The rover uses a custom Pure Pursuit controller to calculate steering commands and reference speed.

Ackermann steering geometry converts the steering command into front-left and front-right wheel steering angles.

Speed is reduced according to steering demand and the rover's distance from the goal.

### 5. Multibody Simulation

The rover is represented using Simscape Multibody.

The tire-force model includes vertical stiffness, vertical damping, longitudinal slip traction, lateral resistance, and friction limiting.

The validated tire damping configuration is:

| Wheel | Vertical damping |
|---|---:|
| Front left | 20 N/(m/s) |
| Front right | 20 N/(m/s) |
| Rear left | 0 N/(m/s) |
| Rear right | 0 N/(m/s) |

## Validated Demonstration Results

The successful Run 5 simulation produced the following results:

| Metric | Result |
|---|---:|
| Simulation duration | 99.98 s |
| Online replans | 4 |
| Final rover position | (47.0072, 35.8527) m |
| Final goal error | 0.1766 m |
| Estimated final speed | 0.00205 m/s |

The route-history visualization includes the initial route, replacement routes, actual rover trajectory, and all six configured obstacles.

## Main Project Files

| File | Purpose |
|---|---|
| `AutonomousNavigationRoughTerrain.prj` | MATLAB Project |
| `setupOffRoadProject.m` | Terrain generation and planning workspace initialization |
| `onlineReplanFromPoseGeometryAware.m` | Geometry-aware A* replanning |
| `runRoughTerrainDemoPipeline.m` | Six-obstacle demonstration workflow |
| `launchElevationTerrainRoverViewerMultiDynamic.m` | Six-obstacle terrain visualization |
| `scenarioConfig_customization_physical_80s.mat` | Six-obstacle scenario configuration |
| `offRoadResults.mat` | Terrain, obstacle, and footprint planning comparison results |
| `OffRoadMultibodyStage1_TireDamping_Cz20_20261101.slx` | Primary validated simulation model |
| `OffRoadMultibodyStage1_Demo_20261101.slx` | Supporting demonstration-pipeline model |
| `OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx` | Earlier development checkpoint |

All core MATLAB scripts and their current workflow dependencies are retained in the project root to preserve existing file references.

## Software Requirements

- MATLAB R2026a
- Simulink
- Simscape
- Simscape Multibody
- Navigation Toolbox
- Image Processing Toolbox

The project also uses MATLAB Function blocks and Stateflow-generated execution components.

## Running the Demonstration

1. Open `AutonomousNavigationRoughTerrain.prj` in MATLAB.
2. Set the project root as the current working directory.
3. Confirm that the required scenario, model, planner, setup, and viewer files are present.
4. Run the following command in the MATLAB Command Window:

```matlab
runRoughTerrainDemoPipeline
```

The demonstration pipeline rebuilds the terrain and planning workspace, loads the six-obstacle scenario, generates the initial route, runs the 100-second Simscape simulation, validates the simulation logs, saves the simulation output, and launches the visualization.

## Results and Data

The repository includes the Run 5 route-history figure and compact planning-comparison data.

The complete Run 5 Simulation Data Dataset is approximately 68.72 MB. It is kept outside normal Git history and should be distributed as a GitHub Release asset.

Dataset filename:

`Demo_Cz20_Run5_100s.mat`

The SDI archive contains an equivalent copy of the Run 5 dataset and is retained locally as a backup.

## Repository Notes

- Generated Simulink cache files (`.slxc`) and build folders (`slprj`) are excluded.
- Temporary diagnostic models, archives, and backups are excluded.
- Large simulation datasets and MATLAB workspace dumps are excluded from normal Git commits.
- Original project files are retained in their root locations to preserve existing workflow dependencies.

## Project Scope

This is a simulation-based autonomous navigation project. The rover, terrain, sensors, tire forces, obstacle interactions, and navigation system are evaluated within MATLAB, Simulink, and Simscape Multibody.
# Microgrid Energy Management

A MATLAB project on energy management for a microgrid with photovoltaic generation and battery storage. It combines MPC scheduling with local corrections for forecast errors.

## Project files

| File | Description |
| --- | --- |
| [run_simulation.m](run_simulation.m) | Daily simulation and comparison of control settings. |
| [MPC.m](control/MPC.m) | Predictive scheduling of battery and grid power. |
| [PLC.m](control/PLC.m) | Local corrections to the scheduled powers. |
| [battery_model.m](model/battery_model.m) | Battery state-of-charge update. |
| [Project report](docs/Microgrid_energy_management.pdf) | Model, control architecture and simulation results. |

## How to use

Open the project folder in MATLAB, add `control/` and `model/` to the MATLAB path, then run `run_simulation.m`.

See the [usage guide](docs/USAGE.md) for the execution steps.

Authors: Matteo De Simone, Giacomo Caiazzo, Marco Bartone and Paola Percuoco.

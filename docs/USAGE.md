# Usage

Open the project folder as MATLAB's Current Folder, then run:

```matlab
addpath('control', 'model')
run_simulation
```

The script generates production and demand profiles, compares two MPC settings and plots the resulting power exchange and battery charge. The supporting functions are called automatically.

The cost vectors `costi_vet` and `costi_vet2` remain in the workspace for comparison.

See the [report](Microgrid_energy_management.pdf) for the complete study.

# Packing Solver
This repository implements a 3D packing problem solver using Mixed Integer Linear Programming (MILP) with Gurobi. The solver finds feasible arrangements of 3D objects inside a container while respecting spatial constraints and orientations.

## Structure

- `packing.py`: The `PackingProblemSolver` class implementing the MILP model and core functionality.
- `input_file.py`: Defines container dimensions, object parameters, and Gurobi configurations.
- `main.py`: The entry point to execute the solution and visualize results.
- `utils.py`: Utility functions for plotting, displaying, and saving solutions.

## Features

- Solve 3D packing problems with constraints and orientations.
- Display solutions as matrices and 3D visualizations.
- Save solutions in `.xlsx` and `.npy` formats.

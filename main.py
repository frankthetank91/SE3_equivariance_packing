#!/usr/bin/env python
# coding: utf-8

# In[1]:


from Package_Solver import PackingProblemSolver
from input_file import get_packing_input
from utils import (
    display_solution_details,
    save_solution_details_to_excel,
    save_raw_solutions_to_npy,
    plot_all_solutions,
    plot_solutions_graphs
)


def main():
    # Load input parameters
    inputs = get_packing_input()
    N, L, W, H = inputs["N"], inputs["L"], inputs["W"], inputs["H"]
    l, w, h = inputs["l"], inputs["w"], inputs["h"]
    wx, wy, wz = inputs["wx"], inputs["wy"], inputs["wz"]

    # Initialize the solver
    solver = PackingProblemSolver(N, L, W, H, l, w, h, wx, wy, wz)
    solver.add_constraints()
    solver.set_constant_objective()

    # Find feasible solutions
    solutions = solver.find_all_feasible_solutions()

    # Display solutions in a readable format
    display_solution_details(solutions, N)

    # Save solutions to files
    save_raw_solutions_to_npy(solutions, filename="raw_solutions.npy")

    # Plot all solutions
    plot_all_solutions(solutions, L, W, H, N)
    plot_solutions_graphs(solutions, N)


if __name__ == "__main__":
    main()


# In[ ]:





#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def display_solution_details(solutions, N):
    """
    Display the `d`, `u`, `s`, and `b` variables for each solution in matrix format.

    :param solutions: List of solutions (each a dictionary of variable names and values).
    :param N: Number of objects.
    """
    for solution_index, solution in enumerate(solutions):
        print(f"\nSolution {solution_index + 1}:")
        d_matrix = [[solution.get(f"d[{i},{j}]", 0) for j in range(6)] for i in range(N)]
        print("\nOrientation matrix (d):")
        for row in d_matrix:
            print(row)

        u_matrix = [[solution.get(f"u[{i},{j}]", 0) for j in range(N)] for i in range(N)]
        s_matrix = [[solution.get(f"s[{i},{j}]", 0) for j in range(N)] for i in range(N)]
        b_matrix = [[solution.get(f"b[{i},{j}]", 0) for j in range(N)] for i in range(N)]

        print("\nAbove matrix (u):")
        for row in u_matrix:
            print(row)

        print("\nSide-by-side matrix (s):")
        for row in s_matrix:
            print(row)

        print("\nBelow matrix (b):")
        for row in b_matrix:
            print(row)


def save_solution_details_to_excel(solutions, filename="solution_details.xlsx"):
    """
    Save solution details to an Excel file.

    :param solutions: List of solutions.
    :param filename: Filename for the Excel file.
    """
    excel_data = [{"Solution": f"Solution {idx + 1}", "Details": str(solution)} for idx, solution in enumerate(solutions)]
    pd.DataFrame(excel_data).to_excel(filename, index=False)


def save_raw_solutions_to_npy(solutions, filename="raw_solutions.npy"):
    """
    Save the raw solutions to a .npy file.

    :param solutions: List of raw solutions.
    :param filename: The name of the .npy file to save.
    """
    np.save(filename, solutions)
    print(f"Solutions saved to {filename}.")


def plot_all_solutions(solutions, L, W, H, N):
    """
    Plot all feasible solutions in a single figure using subplots.

    :param solutions: List of solutions.
    :param L, W, H: Container dimensions.
    :param N: Number of objects.
    :param colors: List of colors for the objects.
    """
    num_solutions = len(solutions)
    colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
    fig = plt.figure(figsize=(20, 10))

    for idx, solution in enumerate(solutions):
        ax = fig.add_subplot(2, (num_solutions + 1) // 2, idx + 1, projection='3d')
        ax.set_xlim([0, L])
        ax.set_ylim([0, W])
        ax.set_zlim([0, H])
        ax.set_xlabel("Length")
        ax.set_ylabel("Width")
        ax.set_zlabel("Height")
        ax.set_title(f"Solution {idx + 1}")

        for i in range(N):
            x = solution[f"x[{i}]"]
            y = solution[f"y[{i}]"]
            z = solution[f"z[{i}]"]
            lx = solution[f"l_hat[{i}]"]
            wy = solution[f"w_hat[{i}]"]
            hz = solution[f"h_hat[{i}]"]

            corners = np.array([
                [x, y, z],
                [x + lx, y, z],
                [x + lx, y + wy, z],
                [x, y + wy, z],
                [x, y, z + hz],
                [x + lx, y, z + hz],
                [x + lx, y + wy, z + hz],
                [x, y + wy, z + hz]
            ])

            faces = [
                [corners[0], corners[1], corners[2], corners[3]],
                [corners[4], corners[5], corners[6], corners[7]],
                [corners[0], corners[1], corners[5], corners[4]],
                [corners[2], corners[3], corners[7], corners[6]],
                [corners[1], corners[2], corners[6], corners[5]],
                [corners[4], corners[7], corners[3], corners[0]]
            ]

            ax.add_collection3d(Poly3DCollection(faces, color=colors[i], linewidths=1, edgecolors='r', alpha=0.5))

    plt.tight_layout()
    plt.show()

def plot_solutions_graphs(solutions, N):
    """
    Plots graph representations for multiple solutions as subplots.

    :param solutions: List of solutions (each a dictionary of variable names and values).
    :param N: Number of objects (nodes).
    :param colors: List of RGBA color tuples for nodes.
    """
    num_solutions = len(solutions)
    colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
    # Calculate grid size for subplots
    cols = min(3, num_solutions)  # Up to 3 columns
    rows = (num_solutions + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(15, 5 * rows), squeeze=False)

    for idx, solution in enumerate(solutions):
        # Determine subplot location
        row, col = divmod(idx, cols)
        ax = axes[row][col]

        # Initialize the graph
        G = nx.Graph()

        # Add nodes with labels corresponding to orientation
        for i in range(N):
            # Find the orientation index where `d[i,j]` equals 1
            orientation_index = next((j for j in range(6) if solution.get(f"d[{i},{j}]", 0) == 1), None)
            G.add_node(i, label=orientation_index, color=colors[i])

        # Add edges with labels corresponding to spatial variables
        for i in range(N):
            for j in range(N):
                if i != j:
                    spatial_variables = {
                        "u": solution.get(f"u[{i},{j}]", 0),
                        "s": solution.get(f"s[{i},{j}]", 0),
                        "b": solution.get(f"b[{i},{j}]", 0),
                    }
                    # Determine which spatial variable is active
                    active_var = next((var for var, val in spatial_variables.items() if val == 1), None)
                    if active_var:
                        G.add_edge(i, j, label=active_var)

        # Position nodes using spring layout
        pos = nx.spring_layout(G)

        # Draw nodes
        node_colors = [G.nodes[node]["color"] for node in G.nodes]
        nx.draw_networkx_nodes(G, pos, ax=ax, node_color=node_colors, node_size=800, alpha=0.8)

        # Draw edges
        nx.draw_networkx_edges(G, pos, ax=ax, edge_color="black", width=1.5, alpha=0.6)

        # Add node labels
        node_labels = {node: G.nodes[node]["label"] for node in G.nodes}
        nx.draw_networkx_labels(G, pos, ax=ax, labels=node_labels, font_size=12, font_color="white")

        # Add edge labels
        edge_labels = {(u, v): G.edges[u, v]["label"] for u, v in G.edges}
        nx.draw_networkx_edge_labels(G, pos, ax=ax, edge_labels=edge_labels, font_size=10)

        # Set subplot title
        ax.set_title(f"Solution {idx + 1}", fontsize=14)
        ax.axis("off")

    # Remove unused subplots if any
    for idx in range(num_solutions, rows * cols):
        row, col = divmod(idx, cols)
        fig.delaxes(axes[row][col])

    # Adjust layout and show the plot
    plt.tight_layout()
    plt.show()


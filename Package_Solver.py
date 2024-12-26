import gurobipy as gp
from gurobipy import GRB
import numpy as np


class PackingProblemSolver:
    def __init__(self, N, L, W, H, l, w, h, wx, wy, wz):
        """
        Initialize the packing problem with given parameters.

        :param N: Number of objects.
        :param L, W, H: Dimensions of the container.
        :param l, w, h: Dimensions of each object.
        :param wx, wy, wz: Weights for the objective (optional).
        """
        self.N = N
        self.L = L
        self.W = W
        self.H = H
        self.l = l
        self.w = w
        self.h = h
        self.wx = wx
        self.wy = wy
        self.wz = wz
        self.model = gp.Model("PackingProblem")

        # Configure solution pool parameters
        self.model.setParam(GRB.Param.PoolSearchMode, 2)
        self.model.setParam(GRB.Param.PoolSolutions, 11)
        self.model.setParam(GRB.Param.PoolGap, 1.0)

        # Decision variables
        self.x = self.model.addVars(N, lb=0, ub=L, vtype=GRB.CONTINUOUS, name="x")
        self.y = self.model.addVars(N, lb=0, ub=W, vtype=GRB.CONTINUOUS, name="y")
        self.z = self.model.addVars(N, lb=0, ub=H, vtype=GRB.CONTINUOUS, name="z")
        self.d = self.model.addVars(N, 6, vtype=GRB.BINARY, name="d")
        self.s = self.model.addVars(N, N, vtype=GRB.BINARY, name="s")
        self.u = self.model.addVars(N, N, vtype=GRB.BINARY, name="u")
        self.b = self.model.addVars(N, N, vtype=GRB.BINARY, name="b")
        self.l_hat = self.model.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name="l_hat")
        self.w_hat = self.model.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name="w_hat")
        self.h_hat = self.model.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name="h_hat")

    def add_constraints(self):
        """
        Add all constraints to the MILP model.
        """
        for i in range(self.N):
            self.model.addConstr(self.d.sum(i, '*') == 1, name=f"orientation_{i}")
            self.model.addConstr(
                self.l_hat[i] == self.d[i, 0] * self.l[i] + self.d[i, 1] * self.l[i] +
                                 self.d[i, 2] * self.w[i] + self.d[i, 3] * self.w[i] +
                                 self.d[i, 4] * self.h[i] + self.d[i, 5] * self.h[i]
            )
            self.model.addConstr(
                self.w_hat[i] == self.d[i, 0] * self.w[i] + self.d[i, 1] * self.h[i] +
                                 self.d[i, 2] * self.l[i] + self.d[i, 3] * self.h[i] +
                                 self.d[i, 4] * self.l[i] + self.d[i, 5] * self.w[i]
            )
            self.model.addConstr(
                self.h_hat[i] == self.d[i, 0] * self.h[i] + self.d[i, 1] * self.w[i] +
                                 self.d[i, 2] * self.h[i] + self.d[i, 3] * self.l[i] +
                                 self.d[i, 4] * self.w[i] + self.d[i, 5] * self.l[i]
            )
            self.model.addConstr(self.x[i] + self.l_hat[i] <= self.L)
            self.model.addConstr(self.y[i] + self.w_hat[i] <= self.W)
            self.model.addConstr(self.z[i] + self.h_hat[i] <= self.H)

        for i in range(self.N - 1):
            for k in range(i + 1, self.N):
                self.model.addConstr(self.s[i, k] + self.u[i, k] + self.b[i, k] == 1)
                self.model.addConstr(self.x[i] - self.x[k] + self.L * self.s[i, k] <= self.L - self.l_hat[i])
                self.model.addConstr(self.y[i] - self.y[k] + self.W * self.b[i, k] <= self.W - self.w_hat[i])
                self.model.addConstr(self.z[i] - self.z[k] + self.H * self.u[i, k] <= self.H - self.h_hat[i])

    def set_constant_objective(self):
        """
        Set a constant objective to focus only on feasibility.
        """
        self.model.setObjective(1, GRB.MINIMIZE)

    def process_solution(self, raw_solution):
        """
        Process a raw solution to round binary variables and cast them to integers.

        :param raw_solution: Dictionary containing raw solution values.
        :return: Processed solution with binary variables rounded to integers.
        """
        return {
            k: int(round(v)) if "d[" in k or "s[" in k or "u[" in k or "b[" in k else v
            for k, v in raw_solution.items()
        }

    def find_all_feasible_solutions(self):
        """
        Find all feasible solutions for the MILP model.
        """
        self.model.optimize()
        solutions = []
        for i in range(self.model.SolCount):
            self.model.setParam(GRB.Param.SolutionNumber, i)
            raw_solution = {v.varName: v.Xn for v in self.model.getVars()}
            solutions.append(self.process_solution(raw_solution))
        return solutions


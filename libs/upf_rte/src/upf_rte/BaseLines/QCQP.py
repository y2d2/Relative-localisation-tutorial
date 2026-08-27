# Solve the following MIP:
#  maximize
#        x +   y + 2 z
#  subject to
#        x + 2 y + 3 z <= 4
#        x +   y       >= 1
#        x, y, z binary

import gurobipy as gp
from gurobipy import *
import numpy as np
import upf_rte.UtilityCode.Transformation_Matrix_Fucntions as TMF

class QCQP:
    def __init__(self, meas_horizon = 10, sigma_uwb=0.1):
        self.meas_horizon = meas_horizon
        self.sigma_uwb = sigma_uwb

        # variables
        self.dt_i_s = np.empty((0, 4))
        self.dt_j_s = np.empty((0, 4))
        self.dij_s = np.empty(0)

        self.t_i = np.empty((0,4))
        self.t_j = np.empty((0,4))

        # Variables for the optimization problem
        self.B = np.empty((0, 9))


    def prune_matrices(self):
        if len(self.dt_i_s) > self.meas_horizon:
            self.dt_i_s = self.dt_i_s[1:]
            self.dt_j_s = self.dt_j_s[1:]
            self.dij_s = self.dij_s[1:]
            self.B = self.B[1:]

    def calculate_trajectories(self):
        self.t_j = np.empty((0, 4))
        self.t_i = np.empty((0, 4))
        prev_T_i = np.eye(4)
        prev_T_j = np.eye(4)
        for i in range(len(self.dt_i_s)):
            DT_i = TMF.transformation_matrix_from_4D_t(self.dt_i_s[i])
            T_i = prev_T_i @ DT_i
            prev_T_i = T_i
            t_i = TMF.get_4D_t_from_matrix(T_i)
            self.t_i = np.append(self.t_i, t_i, axis=0)

            DT_j = TMF.transformation_matrix_from_4D_t(self.dt_j_s[i])
            T_j = prev_T_j @ DT_j
            prev_T_j = T_j
            t_j = TMF.get_4D_t_from_matrix(T_j)
            self.t_j = np.append(self.t_j, t_j, axis=0)


    def calculate_B_matrix(self):
        # See equation 40  in paper.

        self.B = np.empty((0, 9))
        for i in range(len(self.t_i)):
            a_1 =-2 * self.t_i[i][0]
            a_2 =-2 * self.t_i[i][1]
            a_3 = 2 * (self.t_j[i][2]- self.t_i[i][2])
            a_4 = -2 * (self.t_j[i][0] * self.t_i[i][0] +  self.t_j[i][1] * self.t_i[i][1])
            a_5 = 2 * (self.t_j[i][1] * self.t_i[i][0] -  self.t_j[i][0] * self.t_i[i][1])
            a_6 = 2 * self.t_j[i][0]
            a_7 = 2 * self.t_j[i][1]
            a_8 = 1
            # how to calculate v_bar? v = - (2 d_ij n + n^2 )
            # I assume that the expected value of d_ij n = 0 since the expected value of n = 0.
            # However how can we calculate the expected value of n^2?
            # Chat gpt using E[X^2] = Var[X] + E[X]^2  (with X guassian distribution as n)
            # E[X^2] = sigma^2 + 0^2 = sigma^2
            s_i = self.dij_s[i] ** 2 - self.sigma_uwb ** 2
            a_9 = np.linalg.norm(self.t_i[i][:3]) + np.linalg.norm(self.t_j[i][:3]) - 2* self.t_i[i][2] * self.t_j[i][2] - s_i

            t_i = self.t_i[i]
            t_j = self.t_j[i]
            B_i = np.array([a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8, a_9])
            self.B = np.append(self.B, B_i, axis=0)
        print(self.B)

    def update(self, dt_i, dt_j, dij):
        self.dt_i_s = np.append(self.dt_i_s, dt_i, axis=0)
        self.dt_j_s = np.append(self.dt_j_s, dt_j, axis=0)
        self.dij_s = np.append(self.dij_s, dij, axis=0)

        self.prune_matrices()
        self.calculate_trajectories()













#TODO add the correct formules from paper.
if __name__=="__main__":
    # Create a new model
    m = Model("matrix_qcqp")

    # Variables el R^(9x1)
    # tx, ty, tz, cos(theta), sin(theta), tx cos(theta) + ty sin(theta), ty cos(theta) - tx sin(theta), tx^2 + ty^2 + tz^2, 1

    # Create a 2D array of variables
    x = []
    for i in range(5):
        x.append([])
        for j in range(5):
            x[i].append(m.addVar(name="x_" + str(i) + "_" + str(j)))

    # Update model to integrate new variables
    m.update()

    #TODO: Calculate B matrix


    # TODO: Calculate the S matrix

    # Set objective: sum of all x_ij^2
    obj = quicksum(x[i][j] * x[i][j] for i in range(5) for j in range(5))
    m.setObjective(obj, GRB.MINIMIZE)

    # Add constraint: sum of all x_ij >= 1
    m.addConstr(quicksum(x[i][j] for i in range(5) for j in range(5)) >= 1, "c0")

    # Add non-convex quadratic constraint: sum of all x_ij^2 <= 1
    m.addQConstr(quicksum(x[i][j] * x[i][j] for i in range(5) for j in range(5)) <= 1, "qc0")

    # Set to non-convex
    m.Params.NonConvex = 2

    # Optimize model
    m.optimize()

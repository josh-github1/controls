"""Recreating simulation from notes for a Sliding Mode Controller

    Nonlinear control method

    This is meant to be a python version of how I see it being done in Simulink
"""
import scipy
from scipy.integrate import odeint

import matplotlib
from matplotlib.pyplot import *
from typing import List, Any
import math

from sympy import *
import sympy as sym
import numpy as np
from numpy import diff

########################################
###### Defining system parameters ######
########################################

# Time
dt = 0.001
t_final = 3

# Plant coefficients
m = 0.2
c = 0.1
k = 10

# Reference
xd_mag = 0.5    # Magnitude of desired 'x'
xd_w = 10.0       # Frequency

# Disturbances
d_mag = 2.0
d_w = 15.0
D = 1.8         # Disturbance upper bound

# SMC controller gains
lambda_val = 5
eta = 5


def system_input(t: Float):
    """Sinusoidal input into the system."""

    phase = -np.pi/3
    bias = 0
    reference_input = xd_mag*np.sin(xd_w*t + phase ) + bias
    return [reference_input]


def disturbance_input(t: Float):
    """Disturbance to the system."""
    disturbance_input = d_mag*np.sin(d_w*t)
    return disturbance_input


def saturation(v):
    """Saturation or the Sign function in SMC."""

    return v if np.abs(v) <= 1 else np.sign(v)


def SMC_controller(sys_states: List[Float],
                   Ts: float):
    """SMC Controller.
    
    Args:
        sys_commands: Desired states
        sys_states: States of the system
        Ts: Sampling time

    Returns:
        Controller output
    """

    # Desired states
    x_des = system_input(Ts)
    x_dot_des = diff(x_des)/Ts
    x_ddot_des = diff(x_dot_des)/Ts

    # Errors (e, e_dot, e_ddot, etc.)
    # Assuming that X = [x, x_dot, xddot]^T...
    x = sys_states[0]
    x_dot = sys_states[1]
    # x_ddot = sys_states[2]
    e = x_des - x
    e_dot = x_dot_des - x_dot
    # e_ddot = x_ddot_des - x_ddot

    # Lyapunov function, (1/2)*s^2, & derivative:
    s = e_dot + lambda_val*e
    # s_dot = e_ddot + lambda_val*e_dot

    # Controller output. System coefficients were defined above
    u = m*x_ddot_des + c*x_dot + k*x + m*lambda_val*e_dot + eta*s + D*saturation(s)

    return u


def nonlinear_sys_ODE(X: List[Float], Ts: Float):
    """Nonlinear system with SMC & unknown disturbance, d(t).
    
    The original equation was the following:
    m*x_ddot = c*x_dot + k*x = u + d(t)
    where |d(t)| < D
        d(t) is the unknown disturbance
        D is the known upper bound.

    Args:
        X: States of the system
        Ts: Sampling time

    Returns:
        ???
    """

    # Output of SMC Controller
    u = SMC_controller(X, Ts)
    
    disturbance = disturbance_input(Ts)

    x, x_dot = X

    x_ddot = (1/m)*(-c*x_dot - k*x + u + disturbance)

    return [x_dot, x_ddot]

def main():
    print("Starting controller simulation.")

    ########################################
    ####### Setting up variables ###########
    ########################################
    dt = 0.001                          # Timestep
    t_final = 3                         # Final time
    iter = int(t_final/dt)              # Number of iterations for simulation loop
    t = np.linspace(0, t_final, iter)   # Creating time vector
    x_0 = [0.0, 0.0]                        # Initial conditions

    ########################################
    ########## Main simulation loop ########
    ########################################
    curr_t = 0
    for i in range(0, t_final, iter):

        # ts = [t[i-1],t[i]]         # time interval
        curr_t += iter

        # recommended not to use odeint, unless closed loop diff eq.. would want to discretize manually instead
        x = odeint(nonlinear_sys_ODE, x_0, t)

if __name__ == "__main__":
    main()

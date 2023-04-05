"""Recreating simulation from notes for a Sliding Mode Controller

    Nonlinear control method

    This is meant to be a python version of how I see it being done in Simulink
"""
import scipy
from scipy.integrate import ode
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

# Sinusoidal params
phase = -np.pi/3
bias = 0

# Disturbances
d_mag = 2.0
d_w = 15.0
D = 1.8         # Disturbance upper bound

# SMC controller gains
lambda_val = 5
eta = 5


def system_input(t: Float):
    """Sinusoidal input into the system."""
    reference_input = xd_mag*np.sin(xd_w*t + phase ) + bias
    return reference_input


def disturbance_input(t: Float):
    """Disturbance to the system."""
    disturbance_input = d_mag*np.sin(d_w*t)
    return disturbance_input


# def saturation(v):
#    """Saturation or the Sign function in SMC."""
#    return v if (np.abs(v)).any() <= 1 else np.sign(v)


def saturation(v):
    if v > 0:
        return 1
    elif v < 0:
        return -1
    elif v == 0:
        return 0
    else:
        return v

def SMC_controller(sys_states,
                   Ts):
    """SMC Controller.
    
    Args:
        sys_commands: Desired states
        sys_states: States of the system
        Ts: Sampling time

    Returns:
        Controller output
    """

    # # Desired states
    # t= Symbol('t')
    # reference_input = xd_mag*np.sin(xd_w*t + phase ) + bias
    # x_des = lambdify(t, reference_input)
    # # x_des = system_input(Ts)
    # x_des = x_des(Ts)

    # # x_des_prime = reference_input.diff(t)/Ts      # ???? why divide by Ts 
    # x_des_prime = reference_input.diff(t)
    # # x_dot_des = lambdify(t, x_des_prime, 'numpy')
    # x_dot_des = lambdify(t, x_des_prime)
    # x_dot_des = x_dot_des(Ts)    
    
    # # x_des_p_prime = x_des_prime.diff(t)/Ts
    # x_des_p_prime = x_des_prime.diff(t)
    # # x_ddot_des = lambdify(t, x_des_p_prime, 'numpy')
    # x_ddot_des = lambdify(t, x_des_p_prime)
    # x_ddot_des = x_ddot_des(Ts) 
    
    x_des = xd_mag*np.sin(xd_w*Ts + phase) + bias
    x_dot_des = xd_mag*xd_w*np.cos(xd_w*Ts)
    x_ddot_des = -xd_mag*xd_w**2*np.sin(xd_w*Ts)


    # Errors (e, e_dot, e_ddot, etc.)
    # Assuming that X = [x, x_dot, xddot]^T...
    # x = sys_states[0]                             # <<<<< This needs to be corrected
    # x_dot = sys_states[1]                         # <<<<< This needs to..

    # x = sys_states
    # x_dot = sys_states

    x, x_dot = sys_states

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


# def nonlinear_sys_ODE(X: List[Float], Ts: Float):
def nonlinear_sys_ODE(Ts, X):
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
    
    disturbance = disturbance_input(Ts)

    # x, x_dot = X
    x, x_dot = X

    # Output of SMC Controller
    u = SMC_controller(X, Ts)

    x_ddot = (1/m)*(-c*x_dot - k*x + u + disturbance)
    # x_dot = float(x_dot)
    # x_ddot = x_ddot.astype(float)
    return x_dot, x_ddot

def main():
    print("###############################")
    print("###############################")
    print("Starting controller simulation.")
    print("###############################")
    print("###############################")

    ########################################
    ####### Setting up variables ###########
    ########################################
    dt = 0.01                          # Timestep
    t_final = 3                         # Final time
    # iter = int(t_final/dt)              # Number of iterations for simulation loop
    # t = np.linspace(0, t_final, iter)   # Creating time vector
    t0 = 1.1
    x_0 = [5, 10]                        # Initial conditions

    ########################################
    ########## Main simulation loop ########
    ########################################
    # curr_t = 0
    # for i in range(0, t_final, iter):

    #     # ts = [t[i-1],t[i]]         # time interval
    #     curr_t += iter

    #     # recommended not to use odeint, unless closed loop diff eq.. would want to discretize manually instead
    #     # x = odeint(nonlinear_sys_ODE, x_0, t)
        
    solver = ode(nonlinear_sys_ODE).set_integrator('dopri5', nsteps= 1000000, method = 'bdf').set_initial_value(x_0, t0)
    # solver = ode(nonlinear_sys_ODE).set_integrator('dopri5').set_initial_value(x_0, t0)

    data = np.ones((601, 2))
    i = 0
    while solver.successful() and solver.t < t_final:
        solver.integrate(solver.t + dt)
        print(solver.t)	
        X = solver.y

if __name__ == "__main__":
    main()

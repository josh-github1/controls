# Recreating simulation from notes for a Sliding Mode Controller
# ^ Nonlinear control method

from scipy.integrate import odeint
from matplotlib.pyplot import *
from typing import List, Any
import math

from sympy import *
import sympy as sym
import numpy as np
from numpy import diff

#### Defining system parameters ####

# Time
dt = 0.001
t_final = 3

# Plant
m = 0.2
c = 0.1
k = 10

# Reference
xd_mag = 0.5    # Magnitude of desired 'x'
xd_w = 10       # Frequency

# Disturbance
d_mag = 2
d_w = 15
D = 1.8         # Disturbance upper bound

# SMC controller gains
lambda_val = 5
eta = 5

def system_input(t: Float):
    # Hardcoding in Phase and Bias
    phase = -np.pi/3
    bias = 0
    reference_input = xd_mag*np.sin(xd_w*t + phase ) + bias
    return reference_input

def saturation(v):
    return v if np.abs(v) <= 1 else np.sign(v)

def SMC_controller(sys_coeff: List[float], 
                   sys_commands: List[float],
                   sys_states: List[Float],
                   Ts: float):
    """SMC Controller.
    
    Args:
        lambda_in: Gain 1
        eta_in: Gain 2
        sys_coeff: Coefficient values
        sys_commands: Desired states
        sys_states: States of the system
        Ts: Sampling time

    Returns:
        Controller output
    """
    # Variable to define system order 
    # probably not needed
    sys_order = len(sys_coeff)

    # Desired states
    x_des = sys_commands[0]
    x_dot_des = diff(x_des)/Ts
    x_ddot_des = diff(x_dot_des)/Ts

    # Errors (e, e_dot, e_ddot, etc.)
    # Assuming that X = [x, x_dot, xddot]^T...
    x = sys_states[0]
    x_dot = sys_states[1]
    x_ddot = sys_states[2]
    e = x_des - x
    e_dot = x_dot_des - x_dot
    e_ddot = x_ddot_des - x_ddot

    # Lyapunov function, (1/2)*s^2, & derivative:
    s = e_dot + lambda_val*e
    s_dot = e_ddot + lambda_val*e_dot

    # Controller output
    u = m*x_ddot_des + c*x_dot + k*x + m*lambda_val*e_dot + eta*s + D*saturation(s)

    return u


def main():
    print("Starting controller simulation.")

    # Time
    dt = 0.001
    t_final = 3
    
    iter = int(t_final/dt)

    # Main simulation loop:
    for i in range(iter):
        # WIP

if __name__ == "__main__":
    main()


########## Delete this stuff #####################   
def sys_ode(x: List[float], t: float):
    """System ODE - it's only one diff EQ."""
    hx = []                               # initializing system ODE to return
    u = feedback_control(x, alpha, t)     # initializing the controller value
    h = x[0]                              # this should be the current height
    A_h = tank_cross_sectional_area(h, r_tank)
    gh = g*h*2.0
    root = 0
    if gh >= 0:
        root = math.sqrt(gh)
    else:
        print('val is < 0')
    print('This is the value of sqrt(2*g*h): ', root)
    hx_1 = (u - a*root) / A_h
    return hx_1


def feedback_control(x: float, gain: float, t: float):
    """Feedback linearizing controller input."""
    alpha = gain

    # May not need state transform for this problem??
    # z = state_transform(h)
    # State transformation
    if t==0:
        h_d = 0.6
    # z_d = state_transform(h_d)     # x_d is the 'desired final height' or 0.6 meters
    
    h_d = 0.6
    h = x    # I think this is the current height in the state vector
    A_h = tank_cross_sectional_area(h, r_tank)
    v = alpha*(h_d - h)
    gh = g*h*2.0
    root = 0
    if gh >= 0:
        root = math.sqrt(gh)
    else:
        print('val is < 0')
    print('This is the value of sqrt(2*g*h): ', root)
    # root = math.sqrt(gh)
    u = a*root-A_h*v
    return u

### Simulating Response

from scipy.integrate import odeint
from numpy import linspace

t0 = 0
tf = 20
N = 1E3
t = linspace(t0, tf, int(N))
h1_0 = h_init
h0 = [h1_0]

# set feedback gain(s)
gains = {'k1': alpha}

h = odeint(sys_ode, h_init, t)
x_1 = h[:,0]

from matplotlib.pyplot import *
plot(t, x_1, 'r', linewidth=2.0, label = r'')
grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
grid(True)
legend()
xlim([t0, tf])
ylabel(r'State ')
xlabel(r'Time  (s)')
show()

plot(x_1, 'r', linewidth=2.0)
grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
grid(True)
xlabel(r'State ')
show()
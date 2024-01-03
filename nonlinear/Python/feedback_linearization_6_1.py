# Problem 6.1 simulation from Applied Nonlinear Control textbook by Slotine et al.

from scipy.integrate import odeint
from matplotlib.pyplot import *

from typing import List, Any

import math

# Defining system parameters:

m = 1           # mass
g = 9.81        # gravitation acceleration
r_tank = 0.5    # tank radius of 0.5 meters
r_out = 0.1     # outlet opening
h_init = 0.1      # initial height, meters
h_d = 0.6      # final height, meters
a = math.pi*(r_out**2)      # cross section of outlet pipe


alpha = 1       # positive constant

def tank_cross_sectional_area(height: float, radius: float):
    """Returns the cross sectional area of a tank at given height. 
    Params:
       height: The current height or liquid level
       radius: Radius of the tank
    Returns:
       float consisting of cross sectional area
    """

    A_h = math.pi*(2*height*radius - height**2)
    return A_h


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
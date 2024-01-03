from scipy.integrate import odeint
from matplotlib.pyplot import *

from typing import List, Any

import math

from scipy.integrate import odeint
from numpy import linspace


def 

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

"""
WIP

x_ddot(t) + k*x(t) = u(t) + d*sin(10*math.pi*t)

k_hat_0 = 90
d_hat_0 = 2

k_actual = 95
d_actual = 4

Note(jramayrat): This wasn't specified in the exercise, but I'm going to assume that the
reference / desired position is a sinusoidal signal (as I've seen it in Slotine's text)

  ref_in(t) = sin(4*t)

  x_ref[k+2] - 2*x_ref[k+1] + x_ref[k] / dt**2 + k*x_ref[k] = ref_in(t) + d*sin(10*math.pi*k)

  x_ref[k+2] = 2*x_ref[k+1] - x_ref[k] + (- k*x_ref[k] + ref_in(t) + d*sin(10*math.pi*k))*dt**2 

"""


from matplotlib.pyplot import *
from typing import List, Any

import math

from numpy import linspace
import numpy as np

def main():

  ########################################
  ###### Defining system parameters ######
  ########################################

  # Time
  t = np.linspace(0, 10, 1002)
  t_curr = t[1]
  t_prev = t[0]

  # Reference signal and output
  ref = np.sin(4*t)
  x_ref = np.zeros(1002)
  x_ref[0] = 0
  x_ref[1] = 1

  # Actual Plant Coefficients. In a real system, we wouldn't 
  # actually know these values, but for the sake of the simulation
  # we are assuming these parameters are known.
  k_actual = 95
  d_actual = 4

  # Estimated starting plant coefficients
  k_est = 90
  d_est = 2

  gain_lambda = 5
  gain_eta = 5

  # Array declarations to store errors, outputs, and inputs for the simulation.
  error_system      = []
  error_k_param     = []
  error_d_param     = []
  controller_inputs = []




  ################################################
  ####### Simulation for adaptive controller #####
  ################################################
  for k in range(1, len(t)-1):
    # Updating time variables
    t_curr = t[k]
    dt = t_curr - t_prev
    t_prev = t_curr

    # Reference model output
    x_ref[k+2] = 2*x_ref[k+1] - x_ref[k] + (- k_actual*x_ref[k] + ref[k] + d_actual*np.sin(10*math.pi*k))*dt**2 

    curr_k_param_error = k_actual - k_est
    curr_d_param_error = d_actual - d_est

    x_ddot_desired = x_ref[k+2] - 2*x_ref[k+1] + x_ref[k] / dt**2

    controller_input = x_ddot_desired + k_est*x_ref[k+2] - d_est*np.sin(10*math.pi*k) + gain_lambda*e_dot + gain_eta*
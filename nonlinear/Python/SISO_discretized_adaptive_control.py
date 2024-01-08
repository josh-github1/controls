"""
WIP Adaptive Control Simulation (equations were done separately on paper)

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
  x_ref[0] = 0  # arbitrary initial condition
  x_ref[1] = 1  # arbitrary initial condition

  # Simulation signal and output (this could be 'actual' signal and output in a real world system)
  x_sim = np.zeros(1002)
  x_sim[0] = 2  # arbitrary initial condition
  x_sim[1] = 3  # arbitrary initial condition

  # Actual Plant Coefficients. In a real system, we wouldn't 
  # actually know these values, but for the sake of the simulation
  # we are assuming these parameters are known.
  k_actual = 95
  d_actual = 4


  # Estimated starting plant coefficients
  k_est = 90
  d_est = 2
  k_est_init = 90
  d_est_init = 2

  # Lambda and Eta gain values. I gave these arbitrary values of 5 and 5 as a start.
  gain_lambda = 5
  gain_eta = 5

  # Estimated coefficient gain values. Arbitrary values were given as a start.
  gain_K_coeff = 5
  gain_D_coeff = 5

  # Array declarations to store errors, outputs, and inputs for the simulation.
  error_system      = []
  error_k_param     = []
  error_d_param     = []
  controller_inputs = []

  e_curr = 0
  e_prev = 0

  x_ref_curr = 0
  x_sim_curr = 0

  ################################################
  #### Simulation for adaptive controller ########
  ################################################
  for i in range(1, len(t)-1):
    # Updating time variables
    t_curr = t[i]
    dt = t_curr - t_prev
    t_prev = t_curr

    # Reference model output
    x_ref[i+2] = 2*x_ref[i+1] - x_ref[i] + (- k_actual*x_ref[i] + ref[i] + d_actual*np.sin(10*math.pi*i))*dt**2 

    curr_k_param_error = k_actual - k_est
    curr_d_param_error = d_actual - d_est

    x_ref_curr = x_ref[i+2]

    e_curr = x_ref_curr - x_sim_curr

    ## Computing for parameters and values that go into the controller equation.
    x_ddot_desired = x_ref[i+2] - 2*x_ref[i+1] + x_ref[i] / dt**2
    e_dot = (e_curr - e_prev) / dt
    s = e_dot + gain_lambda*e_curr

    # Parameter adaptation integrals
    integral_k += s*x_sim*dt
    integral_d += -np.sin(10*math.pi*i)*s*dt

    # Adaptation laws
    k_est = k_est_init + gain_K_coeff*integral_k
    d_est = d_est_init + gain_D_coeff*integral_d

    # Equation for adaptive controller
    controller_input = x_ddot_desired + k_est*x_ref[i+2] - d_est*np.sin(10*math.pi*i) + gain_lambda*e_dot + gain_eta*s

    # Output of the simulated system, provided with controller and estimated parameters computed through adaptation laws.
    x_sim[i+2] = 2*x_sim[i+1] - x_sim[i] + (- k_est*x_sim[i] + controller_input + d_est*np.sin(10*math.pi*i))*dt**2
    x_sim_curr = x_sim[i+2]

    e_prev = e_curr
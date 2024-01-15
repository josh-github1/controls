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

  (x_ref[i+1] - 2*x_ref[i] + x_ref[i-1]) / dt**2 + k*x_ref[k] = ref_in(t) + d*sin(10*math.pi*k)

  Reference: https://charlesreid1.com/wiki/Basic_discretization_techniques

"""


from matplotlib.pyplot import *
from typing import List, Any

import math
import matplotlib.pyplot as plt

from numpy import linspace
import numpy as np


# Projection operator to keep parameter adaptations bounded. 
def projection_operator(param_max: float, 
                        param_min: float, 
                        curr_param_val: float,
                        z: float):
  if (curr_param_val >= param_max and z > 0):
    return 0
  if (curr_param_val <= param_min and z < 0):
    return 0
  else:
    return z
  


def main():

  ########################################
  ###### Defining system parameters ######
  ########################################
  n = 10000
  # Time
  t = np.linspace(0, 10, n)
  t_curr = t[1]
  t_prev = t[0]


  # Reference signal and output

  x_ref = np.sin(2*math.pi*t + math.pi/3)

  x_ref[0] = 0  # arbitrary initial condition
  x_ref[1] = 0  # arbitrary initial condition

  # Simulation signal and output (this could be 'actual' signal and output in a real world system)
  x_sim = np.zeros(n)
  x_sim[0] = 0  # arbitrary initial condition
  x_sim[1] = 0  # arbitrary initial condition

  # Actual Plant Coefficients. In a real system, we wouldn't 
  # actually know these values, but for the sake of the simulation
  # we are assuming these parameters are known.
  k_actual = 95
  d_actual = 4


  # Estimated starting plant coefficients
  k_est_init = 90
  d_est_init = 2
  k_est = k_est_init
  d_est = d_est_init

  # Lambda and Eta gain values. I gave these arbitrary values of 5 and 5 as a start.
  gain_lambda = 10
  gain_eta = 10

  # Estimated coefficient gain values. Arbitrary values were given as a start.
  gain_K_coeff = 5
  gain_D_coeff = 5




  # Array declarations to store errors, outputs, and inputs for the simulation.
  e_arr             = []
  e_dot_arr         = []
  error_k_param     = []
  error_d_param     = []
  controller_inputs = []
  k_est_values      = []
  d_est_values      = []
  x_d_arr           = []
  x_arr             = []

  e_curr = 0
  e_prev = 0

  x_ref_curr = 0
  x_sim_curr = 0

  integral_k = 0
  integral_d = 0

  print("Starting sim")
  ################################################
  #### Simulation for adaptive controller ########
  ################################################
  for i in range(1, n-1):
    # print("At index ", i)
    # Updating time variables
    t_curr = t[i]
    dt = t_curr - t_prev
    t_prev = t_curr

    # Reference model output. I think this is x_d(t) for the assignment
    x_ref[i] = np.sin(2*math.pi*t[i] + math.pi/3)

    # ^^^ Need t

    curr_k_param_error = k_actual - k_est
    curr_d_param_error = d_actual - d_est

    # Appending estimated parameter values to arrays for plotting
    error_k_param.append(curr_k_param_error)
    error_d_param.append(curr_d_param_error)
    k_est_values.append(k_est)
    d_est_values.append(d_est)



    e_curr = x_ref[i] - x_sim[i]

    ## Computing for parameters and values that go into the controller equation.
    x_ddot_desired = (x_ref[i+1] - 2*x_ref[i] + x_ref[i-1]) / dt**2
    e_dot = (e_curr - e_prev) / dt
    s = e_dot + gain_lambda*e_curr

    # Debug
    # print("Output of s: ", s)

    # Appending system error values to arrays for plotting
    e_arr.append(e_curr)
    e_dot_arr.append(e_dot)

    #######################################
    #### Adaptation Law eqs. ##############
    #######################################
    # # Parameter adaptation integrals
    # integral_k += s*x_sim_curr*dt
    # integral_d += -np.sin(10*math.pi*i*dt)*s*dt

    # # Adaptation laws without projection op.
    # k_est = k_est_init + gain_K_coeff*integral_k
    # d_est = d_est_init + gain_D_coeff*integral_d

    # Adaptation laws with projection op.
    k_max = 100
    k_min = 80
    z_for_param_k = s*x_sim_curr
    integral_k += projection_operator(k_max, k_min, k_est, z_for_param_k)*dt
    k_est = k_est_init + gain_K_coeff*integral_k

    d_max = 5
    d_min = 0
    z_for_param_d = -np.sin(10*math.pi*i*dt)*s
    integral_d += projection_operator(d_max, d_min, d_est, z_for_param_d)*dt
    d_est = d_est_init + gain_D_coeff*integral_d

    # For debugging
    # print("Output of adaptation law for k_est: ", k_est)
    # print("Output of adaptation law for d_est: ", d_est)
    #######################################

    # Equation for adaptive controller
    controller_input = x_ddot_desired + k_est*x_ref[i] - d_est*np.sin(10*math.pi*i*dt) + gain_lambda*e_dot + gain_eta*s


    #   (x_ref[i+1] - 2*x_ref[i] + x_ref[i-1]) / dt**2 + k*x_ref[k] = ref_in(t) + d*sin(10*math.pi*k)

    # Output of the simulated system, provided with controller and estimated parameters computed through adaptation laws.
    x_sim[i+1] = 2*x_sim[i] - x_sim[i-1] + (-k_actual*x_sim[i] + controller_input + d_actual*np.sin(10*math.pi*i*dt))*dt**2
    
    
    x_sim_curr = x_sim[i+1]
    x_arr.append(x_sim_curr)

    e_prev = e_curr



  e_arr.append(e_curr)
  ####### Section for plotting ##############

  plt.plot(t[:n-1], x_sim[:n-1], label='actual')
  plt.plot(t[:n-1], x_ref[:n-1], label='reference')
  plt.plot(t[:n-1], e_arr[:n-1], label='e_arr')
  plt.xlabel('Time')
  plt.legend()
  plt.show()

  plt.plot(t[:n-1], k_est_values[:n-1], label='Estimated k parameter values')
  plt.title('Estimated K values')
  plt.xlabel('Time')
  plt.legend()
  plt.show()


  plt.plot(t[:n-1], d_est_values[:n-1], label='Estimated d parameter values')
  plt.title('Estimated D values')
  plt.xlabel('Time')
  plt.legend()
  plt.show()

  plt.plot(e_dot_arr, e_arr, label='x_axis: e_dot | y_axis: e')
  plt.title('Phase Trajectory')
  plt.ylabel('e')
  plt.xlabel('e_dot')
  plt.legend()
  plt.show()


if __name__ == "__main__":
  main()







    # Don't use _curr or _prev for naming. Actually makes it more confusing
    # x_ref_curr = x_ref[i]

    # Reference model output. I think this is x_d(t) for the assignment
    # x_ref[i+2] = 2*x_ref[i+1] - x_ref[i] + (-k_actual*x_ref[i] + ref[i] + d_actual*np.sin(10*math.pi*i))*dt**2 
    # x_d_arr.append(x_ref[i])
"""
#####################################################################
Example (see Prof. Bashash's email from Jan 4 2024)
#####################################################################

P(s) = 1/(s+5)      (Time domain: x_dot(t) + 5*x(t) = u(t)) 
C(s) = kp + ki/s + kd*s,   (Time domain: u(s) = kp*e(t) + ki*int_e(t) + kd*e_dot(t))
where kp = 1; ki = 10; kd = 0.1

discretized:
(x[k+1] - x[k]) / dt = -5*x[k] + u[k]
x[k+1] = (-5*x[k] + u[k])*dt + x[k]


"""


import numpy as np
import matplotlib.pyplot as plt
import math

def main():
  print("Starting Controller simulations...")


###############################################
########### PART 1 - STEP RESPONSE ############
###############################################

  x = np.zeros(1002)
  x[0] = 0
  x[1] = 0
  t = np.linspace(0, 10, 1002)

  ###### array to store error ######
  e_arr = np.zeros(1002)
  u = np.zeros(1002)

  # Creating step signal
  step = np.linspace(0, 10, 1002)
  step[0:100] = 0
  step[100:] = 1

  # PID Controller values
  kp = 1
  ki = 10
  kd = 0.1

  e_curr = 0
  e_prev = 0

  proportional_term = 0
  total_integral = 0
  integral_term = 0
  derivative_term = 0

##################################################
##### Step response controller simulation ########
##################################################
  for i in range(0, len(t)-2):
    dt = t[i+1] - t[i]

    e_curr = step[i] - x[i]

    # For plotting
    e_arr[i] = e_curr

    proportional_term = kp*e_curr

    # Adding in integral & derivative term
    if i > 1:
      print("integral term") 
      total_integral += e_curr*dt
      integral_term = ki*total_integral
      print("derivative term")
      derivative_term = kd*(e_curr-e_prev)/dt

    u[i] = proportional_term + integral_term + derivative_term

    e_prev = e_curr

    k = 5

    # Output of simulated system:
    x[i+1] = (-k*x[i] + u[i])*dt + x[i]


  fig, axes = plt.subplots(1)

  axes.plot(t, x, label='output')
  axes.plot(t, step, label='step')
  axes.set_title('Setpoint and Actual output')
  axes.set_ylabel('Amplitude')
  axes.set_xlabel('Time (seconds)')

  # axes[1].plot(t, e_arr, label='error')
  # axes[1].set_title('Error')

  # axes[2].plot(t, u, label='system input')
  # axes[2].set_title('Input')

  plt.legend()
  plt.show()

################################################
############ PART 2 - SINE WAVE ################
################################################
  
  x = np.zeros(1002)
  x[0] = 0
  x[1] = 0
  t = np.linspace(0, 10, 1002)

  ###### array to store error ######
  e_arr = np.zeros(1002)
  u = np.zeros(1002)

  ### Reference signal - sine wave ####
  t = np.linspace(0, 10, 1002)
  r = 2*np.sin(0.2*2*math.pi*t) + 5*np.sin(0.3*2*math.pi*t)

  # PID Controller values
  kp = 1
  ki = 10
  kd = 0.1

  e_curr = 0
  e_prev = 0

  proportional_term = 0
  total_integral = 0
  integral_term = 0
  derivative_term = 0
  ##### Controller Sim for sine wave#############
  for i in range(0, len(t)-2):
    dt = t[i+1] - t[i]

    e_curr = r[i] - x[i]

    # For plotting
    e_arr[i] = e_curr

    proportional_term = kp*e_curr

    # Adding in integral & derivative term
    if i > 1:
      print("integral term") 
      total_integral += e_curr*dt
      integral_term = ki*total_integral
      print("derivative term")
      derivative_term = kd*(e_curr-e_prev)/dt

    u[i] = proportional_term + integral_term + derivative_term

    e_prev = e_curr

    k = 5

    # Output of simulated system:
    x[i+1] = (-k*x[i] + u[i])*dt + x[i]


  fig, axes = plt.subplots(1)

  axes.plot(t, x, label='output')
  axes.plot(t, r, label='step')
  axes.set_title('Setpoint and Actual output')
  axes.set_ylabel('Amplitude')
  axes.set_xlabel('Time (seconds)')

  plt.legend()
  plt.show()
  
if __name__ == "__main__":
  main()



"""
References:
  https://thingsdaq.org/2022/04/07/digital-pid-controller/

"""
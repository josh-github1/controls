"""

Example 1:

x_dot + ax = u;
u = kp*e
x(k+1) = x(k)*(1-a*dt) + u(k)*dt


Example 2:
m*x_ddot + c*x_dot + k*x = u
u = kp*e + ki*integral(e) + kd*derivative(e)

## Discretized version of example 2 ODE: 
m * (x[k+2] - 2*x[k+1] + x[k]) / dt^2   +     c * (x[k+1]-x[k]) / dt      +    k * x[k]    =     u[k]

x[k+2] = 2*x[k+1] - x[k] - c*dt*( x[k+1] - x[k] )/m  -   (dt^2/m)*x[k] + (dt^2/m)*u[k]

"""


import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import math


def main():
  print("Starting Controller simulations...")
  
# ###############################################
# ########### PART 1 - SINE WAVE ################
# ###############################################

  ### Reference signal - sine wave ####
  t = np.linspace(0, 10, 1002)
  r = 2*np.sin(0.2*2*math.pi*t) + 5*np.sin(0.3*2*math.pi*t)
  
  ###### diff eq. values ###########
  K = 5

  ###### PID Controller values #####
  kp = 10
  ki = 1
  kd = 1
  error_integral = 0

  ###### array to store outputs ####
  x = np.zeros(1002)
  x[1] = 0.5

  ###### array to store error ######
  e_arr = []

  ##### Controller Sim for sine wave#############
  for k in range(1, len(t)-1):
    dt = t[k+1] - t[k]

    e_curr = r[k] - x[k]
    e_arr.append(e_curr)

    u = kp*e_curr

    # Adding in integral & derivative term
    if k > 1:

      print("integral term added")
      error_integral += (e_curr-e_arr[k-1])*dt
      u += ki*error_integral

      print("derivative term added")
      u += kd*(e_curr-e_arr[k-1])/dt

    x[k+1] = x[k]*(1-K*dt) + u*dt
  
  plt.plot(t, x, label='output')
  plt.plot(t, r, label='reference')
  plt.legend()
  plt.show()


###############################################
########### PART 2 - STEP RESPONSE ############
###############################################

  x = np.zeros(1002)
  x[0] = 0
  x[1] = 0
  t = np.linspace(0, 10, 1002)
  ###### array to store error ######
  e_arr = []

  # Creating step signal
  step = np.linspace(0, 10, 1002)
  step[0:100] = 0
  step[100:] = 5

  # PID Controller values
  kp = 1
  ki = 10
  kd = 1
  error_integral = (step[0]-x[0])*(t[1]-t[0])
  e_curr = 0
  e_prev = 0

  ##### Controller Sim for step #############
  for k in range(1, len(t)-2):
    dt = t[k+1] - t[k]

    e_curr = step[k] - x[k]
    e_arr.append(e_curr)

    u = kp*e_curr

    # Adding in integral & derivative term
    if k > 1:

      # minseg, simulink 
      print("integral term added")      # can't keep this in real hardware, realtime.. previous error is current error.. saving as single variable without entire history, look at me285 lecture notes
      # error_integral += e_arr[k-1]*dt
      error_integral += e_prev*dt
      u += ki*error_integral

      print("derivative term added")
      # u += kd*(e_curr-e_arr[k-1])/dt
      u += kd*(e_curr-e_prev)/dt

    e_prev = e_curr

    c = 1
    m = 1
    k = 1

    ###### These following lines of code are what give the output of our simulated system. 
    x[k+1] = x[k]*(1-K*dt) + u*dt
    # x[k+2] = 2*x[k+1] - x[k] - c*dt*( x[k+1] - x[k] )/m  -   (dt**2/m)*x[k] + (dt**2/m)*u
    # x[k+2] = 2*x[k+1] - x[k] - (dt**2)*(c*(x[k+1] - x[k])/dt  +  k*x[k] + u)/m
    # x[k+2] = 2*x[k+1] - x[k] - (c*dt*( x[k+1] - x[k] )/m) - ((dt**2)*k*x[k]/m) + ((dt**2)*u/m)


  plt.plot(t, x, label='output')
  plt.plot(t, step, label='step')
  plt.legend()
  plt.show()

if __name__ == "__main__":
  main()
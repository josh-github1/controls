# PID Simulation for OH w/out odeint

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint


"""
It should be simple, Matlab:

% x_dot + ax = u;
% u = kp*e

% x(k+1) = x(k)*(1-a*dt) + u(k)*dt

t = [0:0.01:10];
r = 2*sin(0.2*2*pi*t) + 5*sin(0.3*2*pi*t);

figure; plot(t,r)

a = 5;
kp = 100;
x(1) = 0.5;

for k = 1:length(t)-1
    dt = t(k+1) - t(k);

    u(k) = kp*(r(k)-x(k));

    x(k+1) = x(k)*(1-a*dt) + u(k)*dt;

end

figure; plot(t,r,t,x)

"""

def explicit_euler_method(plant_ode_converted, y0, t0, t, dt: float):
  """Explicit Euler Method

  Args:
    ode: Diff Eq. to solve for
    y0: initial function output
    t0: initial time
    t: total time to run solver
    dt: time delta

  Returns:

  """
  y_curr = y0
  t_curr = t0
  while (t_curr <= t):
    y_curr = y_curr + dt*plant_ode_converted(y_curr, t_curr)
    t_curr = t_curr + dt
  return y_curr


def rk4(sys_ode,
        x0, 
        y0, 
        xn, 
        n):
  """Classic Runge-Kutta Method.

  The xn's would be the next point in time
  that's found in the for loop iteration (I think).

  Args:

  Returns:

  """

  h = (xn-x0)/n       # step size
  
  print('x0\ty0\tyn')
  for i in range(n):
    
    k1 = h * (sys_ode(x0, y0))
    k2 = h * (sys_ode((x0+h/2), (y0+k1/2)))
    k3 = h * (sys_ode((x0+h/2), (y0+k2/2)))
    k4 = h * (sys_ode((x0+h), (y0+k3)))

    k = (k1 + 2*k2 + 2*k3 + k4) / 6
    
    yn = y0 + k
    
    print('%.4f\t%.4f\t%.4f'% (x0,y0,yn) )
    print('-------------------------')
    
    y0 = yn
    x0 = x0+h
  
  print('\nAt x=%.4f, y=%.4f' %(xn,yn))

  return yn


# 2*w_dot + 5*w = v
# w --> wd
# v = k*(wd-w)
# Do P controller 1st then PI controller
def plant_ode_converted(y: float, u: float):
  """Converted ODE from office hours."""

  return (-5/2)*y + (1/2)*u


def PID_controller(t,
                   delta_t,
                   Kp,
                   process_var,
                   set_point,
                   Ki = None,
                   integral_error = None,
                   Kd = None,
                   process_var_prev = None):
  """PID Controller.
  
  In the control model, the PID block takes error as input
  and outputs the controller output.

  Args:
    t: current time
    delta_t: time delta or sampling period
    Kp: Proportional gain
    process_var: Output process variable (from solving for the system ODE)
    set_point: Desired process var value
    Ki: Integral gain
    integral_error: Last integral error value (from the last PID computation)
    Kd: Derivative gain
    process_var_prev: Previous output process variable (from solving for the system ODE)

  Returns:
    controller output, u
    integral_error, ie
  """
  
  error = set_point - process_var
  print("The current error is: " + str(error))
  proportional_component = Kp*error
  integral_component = 0
  derivative_component = 0

  # if time is >= 1 second, then there should be
  # previous values for computing the integral
  # and derivative component
  # 4-10-23 Update: should change this according to 
  #                 the time step, instead of 1 sec
  # if (t >= 1).any():
  if t >= 1:
    if Ki and integral_error:
      integral_error = integral_error + error*delta_t 
      integral_component = Ki * integral_error
    if Kd and process_var_prev:
      process_derivative = (process_var - process_var_prev) / delta_t
      derivative_component = Kd * process_derivative
  
  u = proportional_component + integral_component + derivative_component

  return u, error, integral_error

global_error = []
# WIP
def sys_ode(t, y):
  """System ODE with controller to solve with RK4.
  
  We should be providing these with 

  Args:
    t: current time
    y: output var

  """
  
  Kp = 1  # need to update this value
  Ki = 20
  Kd = 5

  u, error, integral_error = PID_controller(t, .01, Kp, y, 10, Ki)
  sys_ode_output = plant_ode_converted(y, u)
  global_error.append(error)
  return sys_ode_output


def main():
  print("Starting Controller sim. ")

  total_time = np.linspace(0, 5, 100)
  # delta_t = .01
  output_list_1 = []
  output_list_2 = []

  x = 0
  y = 0
  index = 0
  IC = [0]
  y_des = 10
  integral_error = 0    # initial integral error
  # 4-10-23: can't make sense of the output 
  for i in total_time:
    # next = index + 1
    # y1 = rk4(sys_ode, i, y, total_time[next], 100)
    # output_list_1.append(y1)

    ########## Testing out explicit euler ##############

    curr_y_sim = y
    curr_x_sim = x

    # PID Controller section:
    Kp = 10
    Ki = 5
    Kd = 5
    

    u = PID_controller(i, .01, Kp, curr_y_sim, y_des)
    
    y1 = explicit_euler_method(sys_ode, x, y, i, .01)
    error = y_des - y1
    output_list_1.append(y1)

  plt.plot(total_time, output_list_1)
  plt.show()



if __name__ == "__main__":
  main()
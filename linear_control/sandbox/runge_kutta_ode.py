# https://scicomp.stackexchange.com/questions/36162/python-evaluating-a-second-order-ode-with-rk4
# Author: Carlos eduardo da Silva Lima
# Solving EDO initial value problem (IVP) via scipy and 4Order Runge-Kutta
# 4Order Runge-Kutta

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
    
# Initial conditions
t_initial = 0.0
t_final = 50.0
y0  = 1.0
u0  = 3.0
N   = 10000
h   = 1e-3 # Step

# Enter the definition of the set of ordinary differential equations
def ode(t,y,u):
  ode_1 = u
  ode_2 = -2*y-4*u
  return np.array([ode_1,ode_2])

# RK4
t = np.empty(N)
y = np.empty(N); u = np.empty(N)

t[0] = t_initial
y[0] = y0; u[0] = u0

for i in range(0,N-1,1):

  k11 = h*ode(t[i],y[i],u[i])[0]
  k12 = h*ode(t[i],y[i],u[i])[1]

  k21 = h*ode(t[i]+(h/2),y[i]+(k11/2),u[i]+(k12/2))[0]
  k22 = h*ode(t[i]+(h/2),y[i]+(k11/2),u[i]+(k12/2))[1]

  k31 = h*ode(t[i]+(h/2),y[i]+(k21/2),u[i]+(k22/2))[0]
  k32 = h*ode(t[i]+(h/2),y[i]+(k21/2),u[i]+(k22/2))[1]

  k41 = h*ode(t[i]+h,y[i]+k31,u[i]+k32)[0]
  k42 = h*ode(t[i]+h,y[i]+k31,u[i]+k32)[1]

  y[i+1] = y[i] + ((k11+2*k21+2*k31+k41)/6)
  u[i+1] = u[i] + ((k12+2*k22+2*k32+k42)/6)
  t[i+1] = t[i] + h


# Graphics
plt.style.use('dark_background')
plt.figure(figsize=(7,7))
plt.xlabel(r'$t(s)$')
plt.ylabel(r'$y(t)$ and $u(t)$')
plt.title(r'$\frac{d^{2}y(x)}{dt^{2}}+ + 4\frac{dy(x)}{dt} + 2y(x) = 0$ with $y(t_{0} = 0) = 1$ and $\frac{dy(0)}{dt} = 3$')
plt.plot(t,y,'b-o',t,u,'r-o')
plt.grid()
plt.show()
# A lot of thanks to Github user, Thalaivar. His code served as a template
# for trying an SMC controller for a different plant:
# https://github.com/Thalaivar/python/blob/master/control/sliding_mode/slotine_7.1.1.py

import matplotlib.pyplot as chart
import numpy as np
from scipy.integrate import ode
from scipy.integrate import odeint
import math

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

# SMC controller gains
lambda_val = 5
eta = 5

##### gets time-dependant model params #####
def getParams(t, X):
	x1, x2 = X

	m = 3 + 1.5*math.sin(x2*np.tanh(x2)*t)
	c = 1.2 + 0.2*math.sin(x2*np.tanh(x2)*t)

	return [m, c]


# Reference
xd_mag = 0.5    # Magnitude of desired 'x'
xd_w = 10.0       # Frequency
# Sinusoidal params
phase = -np.pi/3
bias = 0

##### gets desired trajectory#####
def getDesiredTraj(Ts):
	xd_ddot = -xd_mag*xd_w**2*np.sin(xd_w*Ts)
	xd_dot = xd_mag*xd_w*np.cos(xd_w*Ts)
	xd = xd_mag*np.sin(xd_w*Ts + phase) + bias

	return [xd, xd_dot, xd_ddot]

##### define sliding surface #####
def getSlidingSurface(t, X):
	x1, x2 = X
	
	x_d = getDesiredTraj(t)
	s = x2 - x_d[1] + lambda_val*(x1 - x_d[0])
	
	return s

##### define signum function #####
def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    elif x == 0:
        return 0
    else:
        return x

##### define model #####
def model(t, X):
	x1, x2 = X
	
	# m, c = getParams(t, X)
	s = getSlidingSurface(t, X)
	
	x_d = getDesiredTraj(t)
	e = x_d[0] - x1
	e_dot = x_d[1] - x2
	u = m*x_d[2] + c*x2 + k*x1 + m*lambda_val*e_dot + eta*s + D*sign(s)

	x1dot = x2
	x2dot = (1/m)*(-c*x1dot - k*x1 + u + d_mag*np.sin(d_w*t))
	return [x1dot, x2dot]

# u = -m_cap*(x_d[2] - gamma*(x2 - x_d[1]))*np.tanh(x_d[2] - gamma*(x2 - x_d[1])) - c_cap*(x2**2) - eta*np.tanh(s)
# x2dot = (u - c*x2*np.tanh(x2)*x2)/m

##### solver ####
def simulateSMC():
	# Time
  dt = 0.001
  t_final = 3
  X0 = [0, 0]
  t0 = 0
  solver = ode(model).set_integrator('dopri5', nsteps= 1000000, method = 'bdf').set_initial_value(X0, t0)
  data = np.ones((3001, 4))
  i = 0

  while solver.successful() and solver.t < t_final:
    solver.integrate(solver.t + dt)
    print(solver.t)	
    X = solver.y
    x_d = getDesiredTraj(solver.t)
    s = getSlidingSurface(solver.t, X)
    x1, x2 = X
    e = x_d[0] - x1
    e_dot = x_d[1] - x2
    u = m*x_d[2] + c*x2 + k*x1 + m*lambda_val*e_dot + eta*s + D*sign(s)
    error = X[0] - x_d[0]

    data[i] = [u, error, x1, x2]
    i = i + 1
  
  return data

data = simulateSMC()

figure = chart.figure()

fig1 = figure.add_subplot(3, 1, 1)
fig1.plot(data[:,0])

fig2 = figure.add_subplot(3, 1, 2)
fig2.plot(data[:,1])

fig3 = figure.add_subplot(3, 1, 3)
fig3.plot(data[:,2], data[:,3])
fig1.grid()
fig2.grid()
fig3.grid()
chart.show()
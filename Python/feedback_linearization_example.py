# Reference: https://github.com/Walid-khaled/Regulation-and-Trajectory-Tracking-of-Flexible-Joint-Link/blob/main/src/main.ipynb

# ^ Walid Khaled writes good code in my opinion

# System parameters 
m = 0.5 # mass of the pendulum bob
g = 9.81 # Gravitational acceleration
L = 0.2 # Length of pendulum beam 
I = 0.01 # Inertia of actuator 
b = 0.08 # Actuator viscous friction (damping) 
k = 100 # Joint stiffness


# Define system ODE: dx/dt = f(x,u)
import numpy as np
def sys_ode(x, t, gains):
  # Find control 
  if reg == 1: 
    u = control(x, gains, t=0)
  else:
    u = control(x, gains, t)
  # Define direvitive
  dx_1 = x[2] 
  dx_2 = x[3]
  ddx_1 = (1/I)*(u-b*x[2]-k*(x[0]-x[1]))
  ddx_2 = (1/(m*(L**2)))*(k*(x[0]-x[1])- m*g*L*np.sin(x[1]))
  dx = dx_1, dx_2, ddx_1, ddx_2
  return dx


# Define state transformation: z = T(x)
def T(x):
  theta_1 = x[0]
  theta_2 = x[1]
  dtheta_1 = x[2]
  dtheta_2 = x[3]
  
  z_1 = theta_2
  z_2 = dtheta_2
  z_3 = ((k/(m*(L**2)))*(theta_1-theta_2))-((g/L)*np.sin(theta_2))
  z_4 = ((k/(m*(L**2)))*(dtheta_1-dtheta_2))-((g/L)*np.cos(theta_2)*dtheta_2)

  return z_1, z_2, z_3, z_4

# Define inner loop controler: u(x, v)
def inner_control(x, v):
  theta_1 = x[0]
  theta_2 = x[1]
  dtheta_1 = x[2]
  dtheta_2 = x[3]

  # Nonlinear feedback 
  c1 = (m*(L**2)*I)/k
  c2 = k/(m*(L**2))
  c3 = g/L
  u = b*dtheta_1 + k*(theta_1-theta_2) + c1*((c2+ c3*np.cos(theta_2))*(c2*(theta_1-theta_2)- c3*np.sin(theta_2)) - c3*np.sin(theta_2)*(dtheta_2**2)) + v 
  return u


# Define outer loop
def control(x, gains, t):
  # Gains for linear part
  k1 = gains['k1']
  k2 = gains['k2']
  k3 = gains['k3']
  k4 = gains['k4']
  # Find state transformation 
  z = T(x)
  if t==0:
    x_d = [np.pi, np.pi, 0, 0]
  else:
    x_d = [np.pi*np.sin(4*np.pi*t), np.pi*np.sin(4*np.pi*t), 4*(np.pi**2)*np.cos(4*np.pi*t), 4*(np.pi**2)*np.cos(4*np.pi*t)]
  z_d = T(x_d) 

  # Linear part of controller 
  v = k1*(z_d[0]-z[0]) + k2*(z_d[1]-z[1]) + k3*(z_d[2]-z[2]) + k4*(z_d[3]-z[3])
  u = inner_control(x, v)
  return u

from scipy.integrate import odeint
from numpy import linspace

# Initial time
t0 = 0 
# Final time 
tf = 10 
# Numbers of points in time span
N = 1E3 
# Create time span
t = linspace(t0, tf, int(N))
# Set initial state
theta1_0 =  0
theta2_0 =  0
dtheta1_0 = 0
dtheta2_0 = 0
x0 = [theta1_0, theta2_0, dtheta1_0, dtheta2_0]

# Set feedback gains
gains = {'k1':15,
         'k2':25,
         'k3':15,
         'k4':5}

# Integrate system "sys_ode" from initial state 

reg = 1
x = odeint(sys_ode, x0, t, args=(gains,)) 
# Set x_1, x_2 to be a respective solution of system states
x_1, x_2 = x[:,0], x[:,1] 

from matplotlib.pyplot import *
plot(t, x_1, 'r', linewidth=2.0, label = r'x_1')
plot(t, x_2, 'b', linewidth=2.0, label = r'x_2')
grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
grid(True)
legend()
xlim([t0, tf])
ylabel(r'State ')
xlabel(r'Time (s)')
show()

plot(x_1, x_2, 'r', linewidth=2.0)
grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
grid(True)
xlabel(r'State ')
ylabel(r'State ')
show()
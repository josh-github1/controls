from scipy import linspace
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

import math

def pendulum(t, z):
    x1, x2 = z

    b = 0.1
    M = 10
    R = 5
    g = 9.81

    f1 = x2
    f2 = -(b/(M*R**2))*x2 - (g/R)*math.sin(x1)
    return [f1, f2]

def van_der_pol_oscillator(t, z):
    x1, x2 = z

    f1 = x2
    f2 = -x1 + (1-x1**2)*x2
    return [f1, f2]

a, b = 0, 10
t = linspace(a, b, 500)

for x0 in range(0, 6):
    for y0 in [0, 3]:
        sol = solve_ivp(van_der_pol_oscillator, [a, b], [x0, y0], t_eval=t)
        plt.plot(sol.y[0], sol.y[1], ":", color="tab:blue")
plt.show()
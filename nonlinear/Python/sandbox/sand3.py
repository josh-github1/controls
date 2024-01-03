import array
from math import pi, sin
import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt

def system_input(t: float,
                 xd_mag: float,
                 xd_w: float,
                 phase: float,
                 bias: float):
    """Sinusoidal input into the system."""
    reference_input = xd_mag*np.sin(xd_w*t + phase ) + bias
    return reference_input


def step_input(t: float,
               xd_mag: float,
               delay: float):
    """Step input to a system."""
    if (t > delay):
        return xd_mag
    else:
        return 0
        

def model(Y: array, t: float):

  # put in arbitrary values for system's input
  # curr_input = system_input(t, 10, 10, 10, 10)
  curr_input = step_input(t, 10, 1)
  Y1 = Y[1]
  Y2 = -95*Y[0] + curr_input + 4*sin(10*pi*t)
  Y_arr = [Y1, Y2]
  return Y_arr

def main():
    a_t = np.arange(0, 25.0, 0.01)

    # Remember Prof. Bashash recommends not using libraries (implement numerical method from scratch)
    asol = integrate.odeint(model, [1, 0], a_t)
    
    
    print(asol)
    plt.figure(1)

    # TODO: Need to extract from the multi-dimensional array, asol.
    plt.plot(a_t, asol[:, 0], 'b')
    plt.plot(a_t, asol[:, 1], 'g')
    plt.show()

if __name__ == '__main__':
    main()   



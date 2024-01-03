###############################################
# ########### PART 1 - SINE WAVE ################
# ###############################################

#   ### Reference signal - sine wave ####
#   t = np.linspace(0, 10, 1002)
#   r = 2*np.sin(0.2*2*math.pi*t) + 5*np.sin(0.3*2*math.pi*t)
  
#   ###### diff eq. values ###########
#   K = 5

#   ###### PID Controller values #####
#   kp = 1
#   ki = 100
#   kd = 10
#   error_integral = 0

#   ###### array to store outputs ####
#   x = np.zeros(1002)
#   x[1] = 0.5

#   ###### array to store error ######
#   e_arr = []

#   ##### Controller Sim for sine wave#############
#   for k in range(1, len(t)-1):
#     dt = t[k+1] - t[k]

#     e_curr = r[k] - x[k]
#     e_arr.append(e_curr)

#     u = kp*e_curr

#     # Adding in integral & derivative term
#     if k > 1:

#       print("integral term added")
#       error_integral += (e_curr-e_arr[k-1])*dt
#       u += ki*error_integral

#       print("derivative term added")
#       u += kd*(e_curr-e_arr[k-1])/dt

#     x[k+1] = x[k]*(1-K*dt) + u*dt
  
#   plt.plot(t, x, label='output')
#   plt.plot(t, r, label='reference')
#   plt.legend()
#   plt.show()
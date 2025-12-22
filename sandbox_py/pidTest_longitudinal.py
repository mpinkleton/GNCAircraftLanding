#PID Control test
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)


#LTI Continuous time system model (longitudinal)
A_lon = np.array([[-0.284, -23.096, 0.0, -0.171],\
       [0.0, -4.117, 0.778, 0.0],\
       [0.0, -33.884, -3.573, 0.0],\
       [0.0, 0.0, 1, 0.0]])  

B_lon = np.array([[20.168],\
       [0.544],\
       [-39.085],\
       [0.000]])

X_0 = np.array([[0.0],[0.05],[0.0],[0.05]])

U = np.array([[0.0]])
    
#pitch PID control
K_q = 0.12
K_th = 0.50
K_z = 0.25

time = 20
dT = 0.01
timesteps = int(time/dT)

#height initially
Z_0 = -400

# storage for plotting
th_list = []
setpt_th_list = []
time_list = []
z_list = []
setpt_z_list = []
delta_list = []


#loop

for i in range(timesteps):
    
    time = i * dT
    __setpt_z = Z_0 + 2.5*time
                
    #state calculation    
    X = X_0 + dT*(A_lon @ X_0 + B_lon @ U)
    
    th = X[3].item()
    
    q = X[2].item()
        
    Z = Z_0 + dT*(-42*np.sin(th))
    
    #setpoint (pitch angle)
    __setpt_th = K_z*(Z-__setpt_z)    
    __setpt_th = max(min(__setpt_th, 0.45), -0.45)
    
    #setpoint (actuator)
    U[0][0] = K_q*q + K_th*(th - __setpt_th)
    
    #log states and inputs
    
    #states
    th_list.append(th)
    setpt_th_list.append(__setpt_th)
    time_list.append(time) 
    z_list.append(Z)
    setpt_z_list.append(__setpt_z)
    #inputs
    delta_list.append(U[0].item()*(57.295))
    
    #update    
    X_0 = X
    
    Z_0 = Z
    
# ---- Plotting ----

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

# Plot 1: theta and theta_D
ax1.plot(time_list, th_list, label=r'$\theta$')
ax1.plot(time_list, setpt_th_list, label=r'$\theta_{D}$', linestyle='--')
ax1.set_ylabel(r'$\theta$')
ax1.set_title(r'$\theta$ and $\theta_{D}$ over Time')
ax1.legend()
ax1.grid(True)

# Plot 2: Z and Z_D
ax2.plot(time_list, z_list, label=r'$Z$')
ax2.plot(time_list, setpt_z_list, color='r', linestyle='--', label=r'$Z_D$')
ax2.set_ylabel(r'$Z$')
ax2.set_xlabel('Time')
ax2.set_title(r'$Z$ and $Z_D$ over Time')
ax2.legend()
ax2.grid(True)

# Plot 3: actuator
ax3.plot(time_list, delta_list)
ax3.set_ylabel(r'$i_H$')
ax3.set_xlabel('Time')
ax3.set_title(r'$i_H$ (Degrees)')
ax3.legend()
ax3.grid(True)


plt.tight_layout()
plt.show()
    
    
    
    
    
    






    
    

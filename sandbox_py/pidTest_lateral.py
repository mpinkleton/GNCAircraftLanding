#PID Control test
import numpy as np
import matplotlib.pyplot as plt

#LTI Continuous time system model (lateral)
A_lat = np.array([[0.430, 0.094, -1.030, 0.237],
        [-67.334, -7.949, 5.640, 0.0],
        [20.533, -0.655, -1.996, 0.0],
        [0.0, 1.0, 0.0, 0.0]])

B_lat = np.array([[0.272, -0.771],
        [-101.845, 33.474],
        [-6.261, -24.363],
        [0.0, 0.0]])

#Initial conditions
X_0 = np.array([[0.0],[0.0],[0.0],[0.0]])
U = np.array([[0.0],[0.0]])
    
#roll PID control
K_p = 0.04
K_phi = 0.35
K_r = 0.16
w_o = 1.8
K_psi = 1.5

time = 25
dT = 0.01
timesteps = int(time/dT)

#heading, rudder initially
psi_0 = 0.10
del_r = 0

# storage for plotting
phi_list = []
setpt_phi_list = []
time_list = []
psi_list = []
setpt_psi_list = []
delta_a_list = []
delta_r_list = []

#loop

for i in range(timesteps):
    
    time = i * dT
    __setpt_psi = 1.55
    
    #state calculation 
    X = X_0 + dT*(A_lat @ X_0 + B_lat @ U)
    
    phi = X[3].item()
    
    p = X[1].item()
    
    r = X[2].item()
    r_old = X_0[2].item()
    dr_dt = (r-r_old)/dT
    
    psi = psi_0 + dT*(r*np.cos(phi))

    #setpoint (roll angle)
    __setpt_phi = K_psi*(__setpt_psi-psi)    
    __setpt_phi = max(min(__setpt_phi, 0.5), -0.5)    
    
    #setpoint (actuator)
    del_a = K_p*p + K_phi*(phi - __setpt_phi)
    del_r = del_r + dT*(K_r*dr_dt - w_o*del_r)
    U[0][0] = del_a
    U[1][0] = del_r
    
    #states
    phi_list.append(phi)
    setpt_phi_list.append(__setpt_phi)
    time_list.append(time)  
    psi_list.append(psi)
    setpt_psi_list.append(__setpt_psi)
    #inputs
    delta_a_list.append(U[0].item()*(57.295))
    delta_r_list.append(U[1].item()*(57.295))
    
    X_0 = X
    
    psi_0 = psi
    
    
# ---- Plotting ----
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# Plot 1: phi and phi_D
ax1.plot(time_list, phi_list, label=r'$\phi$')
ax1.plot(time_list, setpt_phi_list, label=r'$\phi{D}$', linestyle='--')
ax1.set_ylabel(r'$\phi$')
ax1.set_title(r'$\phi$ and $\phi{D}$ over Time')
ax1.legend()
ax1.grid(True)

# Plot 2: psi and psi_D
ax2.plot(time_list, psi_list, label=r'$\psi$')
ax2.plot(time_list, setpt_psi_list, color='r', linestyle='--', label=r'$\psi_D$')
ax2.set_ylabel(r'$\psi$')
ax2.set_xlabel('Time')
ax2.set_title(r'$\psi$ and $\psi_D$ over Time')
ax2.legend()
ax2.grid(True)

# Plot 3: actuator
ax3.plot(time_list, delta_a_list, label=r'$\delta_A$', color='r')
ax3.plot(time_list, delta_r_list, label=r'$\delta_R$', color='b')
ax3.set_ylabel(r'$\delta_{AR}$')
ax3.set_xlabel('Time')
ax3.set_title(r'$\delta_A, \delta_R$ (Degrees)')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.show()
    
    
    
    
    
    
    






    
    

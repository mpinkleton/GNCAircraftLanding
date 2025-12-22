#PID Control test
import numpy as np
import matplotlib.pyplot as plt

#----------------------Helper functions-------------------------------
def findRange(x1,y1,x2,y2):
    return np.sqrt((x2-x1)**2+(y2-y1)**2)

def psi12(x1,y1,x2,y2):
    R_X = x2-x1
    R_Y = y2-y1
    return np.arctan2(R_Y,R_X)

def wrapRad(a):
    if (a > np.pi): 
        a -= 2*np.pi;
    if (a < -np.pi): 
        a += 2*np.pi;
    return a;

#--------------------------------------------------------------------#

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
x = 0.0
y = -600.0
V = 40.0

#Waypoint coordinates
X_WP = [-100.0,-2000.0,-1250.0]
Y_WP = [-700.0,-1600.0,0.0]

#Time horizon and virtual range for trajectory
T_H = 8
R_star = V*T_H

#Synthetic waypoint starting point and heading (fixed for one)
X_ST = X_WP[0]
Y_ST = Y_WP[0]
WP_NEXT = 1
    
#roll PID control
K_p = 0.04
K_phi = 0.35
K_r = 0.16
w_o = 1.8
K_psi = 1.5

time = 105
dT = 0.01
timesteps = int(time/dT)

#heading, rudder initially
psi_0 = psi12(x, y, X_ST, Y_ST)
del_r = 0

# storage for plotting
phi_list = []
setpt_phi_list = []
time_list = []
psi_list = []
setpt_psi_list = []
delta_a_list = []
delta_r_list = []
x_list = []
y_list = []

#loop

for i in range(timesteps):
    
    time = i * dT
    
    #aircraft heading command (towards synthetic waypoint)
    __setpt_psi = psi12(x, y, X_ST, Y_ST)
       
    #propagate aircraft states
    X = X_0 + dT*(A_lat @ X_0 + B_lat @ U)
    phi = X[3].item()
    p = X[1].item()
    r = X[2].item()
    r_old = X_0[2].item()
    dr_dt = (r-r_old)/dT
    psi = psi_0 + dT*(r*np.cos(phi))
    psi = wrapRad(psi)
    x = x + dT*(V*np.cos(psi))
    y = y + dT*(V*np.sin(psi))

    #setpoint (roll angle)
    __setpt_phi = K_psi*(wrapRad(__setpt_psi-psi))    
    __setpt_phi = max(min(__setpt_phi, 0.5), -0.5)    
    
    #setpoint (actuator)
    del_a = K_p*p + K_phi*(phi - __setpt_phi)
    del_r = del_r + dT*(K_r*dr_dt - w_o*del_r)
    U[0][0] = del_a
    U[1][0] = del_r
    
    #propagate waypoint motion
    psi_WP = psi12(X_WP[WP_NEXT-1],Y_WP[WP_NEXT-1],X_WP[WP_NEXT],Y_WP[WP_NEXT])
    R = findRange(x, y, X_ST, Y_ST)
    V_w = V*(R_star/R)
    print(V_w)
    X_ST = X_ST + dT*(V_w*np.cos(psi_WP))
    Y_ST = Y_ST + dT*(V_w*np.sin(psi_WP))
    if findRange(X_ST, Y_ST, X_WP[WP_NEXT], Y_WP[WP_NEXT]) < 10:
        if (WP_NEXT) < len(X_WP) - 1:
            WP_NEXT = WP_NEXT + 1
        
    #states
    phi_list.append(phi*(180/np.pi))
    setpt_phi_list.append(__setpt_phi*(180/np.pi))
    time_list.append(time)  
    psi_list.append(psi*(180/np.pi))
    setpt_psi_list.append(__setpt_psi*(180/np.pi))
    x_list.append(x)
    y_list.append(y)
    #inputs
    delta_a_list.append(U[0].item()*(180/np.pi))
    delta_r_list.append(U[1].item()*(180/np.pi))
    
    #update
    X_0 = X
    psi_0 = psi
    
    
# ---- Plotting ----
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 8))

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

# Plot 3: actuators
ax3.plot(time_list, delta_a_list, label=r'$\delta_A$', color='r')
ax3.plot(time_list, delta_r_list, label=r'$\delta_R$', color='b')
ax3.set_ylabel(r'$\delta_{AR}$')
ax3.set_xlabel('Time')
ax3.set_title(r'$\delta_A, \delta_R$ (Degrees)')
ax3.legend()
ax3.grid(True)

#Plot 4: trajectory
ax4.plot(x_list, y_list, color='r')
ax4.plot(X_WP, Y_WP, label='WP')
ax4.plot(X_ST, Y_ST, marker='x', markersize=12, label=r'$WP_S$')
ax4.set_ylabel('Y (m)')
ax4.set_xlabel('X (m)')
ax4.set_title('Position')
ax4.legend()
ax4.grid(True)

plt.tight_layout()
plt.show()
    
    
    
    
    
    
    






    
    
